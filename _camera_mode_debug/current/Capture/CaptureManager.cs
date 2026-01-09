using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using CameraMode.UserInterface;
using CameraMode.Utilities;
using HarmonyLib;
using I2.Loc;
using Pug.ECS.Hybrid;
using Pug.RP;
using Pug.Sprite;
using PugMod;
using TMPro;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Rendering;
using ShaderIDs = Pug.RP.ShaderIDs;

// ReSharper disable InconsistentNaming

namespace CameraMode.Capture {
	public class CaptureManager : MonoBehaviour {
		private struct TrackedEntityMono {
			public EntityMonoBehaviour EntityMono;
			public float TimeCreated;
			
			public bool WasRecentlyCreated => Time.time < TimeCreated + 0.25f;
		}

		public static CaptureManager Instance { get; private set; }

		public CaptureUI CaptureUI { get; private set; }
		public CaptureProgressUI CaptureProgressUI { get; private set; }
		private static bool CanOpenCaptureUI => !Manager.menu.IsAnyMenuActive()
            && Manager.main.player != null
            && !Manager.main.player.isDyingOrDead
            && Manager.main.player.adminPrivileges >= 1;

		public CaptureBase CurrentCapture { get; private set; }
		public bool IsCapturing => CurrentCapture != null || _endCaptureRoutine != null;
		
		private Action<byte[]> _currentCaptureCallback;
		private Coroutine _currentCaptureRoutine;
		private bool _currentCaptureIsComplete;
		private Coroutine _startCaptureRoutine;
		private Coroutine _endCaptureRoutine;
		private bool _queueCaptureStoppedMessage;
		
		private bool _uiWasDisabled;
		private bool _simulationWasDisabled;
		private int _oldDynamicWaterSetting;
		private float _startTime;
		private bool _pauseVisuals;

		private Entity _loadAreaEntity;

		private CreateGraphicalObjectSystem _createGraphicalObjectSystem;
		private readonly Dictionary<Entity, TrackedEntityMono> _trackedEntityMonos = new();
		
		private void Awake() {
			Instance = this;

			var captureProgressPrefab = Main.AssetBundle.LoadAsset<GameObject>("Assets/CameraMode/Prefabs/CaptureProgressUI.prefab");
			CaptureProgressUI = Instantiate(captureProgressPrefab, Manager.ui.UICamera.transform).GetComponent<CaptureProgressUI>();

			var capturePrefab = Main.AssetBundle.LoadAsset<GameObject>("Assets/CameraMode/Prefabs/CaptureUI.prefab");
			CaptureUI = Instantiate(capturePrefab, Manager.ui.UICamera.transform).GetComponent<CaptureUI>();

			API.Client.OnWorldCreated += () => {
				_createGraphicalObjectSystem = API.Client.World.GetExistingSystemManaged<CreateGraphicalObjectSystem>();
			};
			API.Client.OnObjectSpawnedOnClient += (entity, _, _) => {
				var entityMono = _createGraphicalObjectSystem.entityMonoBehaviourLookup.GetValueOrDefault(entity);

				if (entityMono is not (SlimeBoss or Firefly))
					return;
				
				_trackedEntityMonos.TryAdd(entity, new TrackedEntityMono {
					EntityMono = entityMono,
					TimeCreated = Time.time
				});
			};
			API.Client.OnObjectDespawnedOnClient += (entity, _, _) => {
				_trackedEntityMonos.Remove(entity);
			};
		}

		private void Update() {
			if (IsCapturing && (!CanOpenCaptureUI || !CaptureUI.IsOpen || !Manager.sceneHandler.isInGame))
				StopCapture();
			
			if (CanOpenCaptureUI) {
				if (Input.GetKeyDown(KeyCode.F4)) {
					if (CaptureUI.IsOpen) {
						CaptureUI.Close();
					} else {
						CaptureUI.Open();
					}
				}
				
				// F5 to cycle wait time when Camera Mode UI is open
				if (CaptureUI.IsOpen && Input.GetKeyDown(KeyCode.F5)) {
					CycleWaitTime();
				}
			} else if (CaptureUI.IsOpen) {
				CaptureUI.Close();
			}
			
			UpdateCapture();
			UpdatePauseVisuals();
		}
		
		private static readonly float[] WaitTimeValues = { 0.3f, 0.5f, 0.7f, 1.0f, 1.2f, 1.5f, 2.0f };
		private Coroutine _waitTimeDisplayRoutine;
		
		private void CycleWaitTime() {
			var current = Config.Instance.AreaLoadWaitTime;
			var nextIndex = 0;
			
			for (var i = 0; i < WaitTimeValues.Length; i++) {
				if (current < WaitTimeValues[i] + 0.05f) {
					nextIndex = (i + 1) % WaitTimeValues.Length;
					break;
				}
			}
			
			Config.Instance.AreaLoadWaitTime = WaitTimeValues[nextIndex];
			
			// Show wait time in the title text temporarily
			if (_waitTimeDisplayRoutine != null)
				StopCoroutine(_waitTimeDisplayRoutine);
			_waitTimeDisplayRoutine = StartCoroutine(ShowWaitTimeInTitle());
		}
		
		private IEnumerator ShowWaitTimeInTitle() {
			var titleText = CaptureUI.inCameraModeText;
			if (titleText == null)
				yield break;
			
			// Try to disable localization on this PugText temporarily
			var type = titleText.GetType();
			var localizeField = type.GetField("localize", System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
			var originalLocalize = true;
			if (localizeField != null) {
				originalLocalize = (bool)localizeField.GetValue(titleText);
				localizeField.SetValue(titleText, false);
			}
			
			titleText.Render($"Wait: {Config.Instance.AreaLoadWaitTime:F1}s");
			
			yield return new WaitForSeconds(1.5f);
			
			// Restore localization and render the original text
			if (localizeField != null) {
				localizeField.SetValue(titleText, originalLocalize);
			}
			titleText.Render("Camera Mode");
			_waitTimeDisplayRoutine = null;
		}

		public void Capture(CaptureBase capture, string captureName = null) {
			if (IsCapturing)
				return;
			
			captureName ??= $"Capture {DateTime.Now:yyyy-MM-dd HH.mm.ss}";
			
			CurrentCapture = capture;
			var quality = Config.Instance.CaptureQuality;
			
			// Check if we should use background processing (Fast mode + supported capture type)
			_useBackgroundProcessing = quality.UsesBackgroundConversion() && capture.SupportsBackgroundProcessing;
			
			if (_useBackgroundProcessing) {
				// Fast mode: Capture tiles only, then process everything in background
				_currentTileDataCallback = tileData => {
					_currentCaptureIsComplete = true;
					
					if (tileData != null) {
						// Start background stitching + encoding
						StartCoroutine(BackgroundProcessingRoutine(captureName, tileData));
					}
				};
				_currentCaptureCallback = null;
			} else {
				// Normal mode: Direct encoding
				_currentCaptureCallback = imageData => {
					Utils.WriteCapture(captureName, quality.GetFileExtension(), imageData);
					_currentCaptureIsComplete = true;
				};
				_currentTileDataCallback = null;
			}
			_currentCaptureRoutine = null;
			_currentCaptureIsComplete = false;

			_startCaptureRoutine = StartCoroutine(StartCaptureRoutine());
		}
		
		private bool _useBackgroundProcessing;
		private Action<CapturedTileData> _currentTileDataCallback;
		
		private IEnumerator BackgroundProcessingRoutine(string captureName, CapturedTileData tileData) {
			// Wait for capture UI to close
			yield return null;
			yield return null;
			
			var startTime = Time.realtimeSinceStartup;
			var totalTiles = tileData.TileImages.Count;
			
			Utils.DisplayChatMessage($"Background: Stitching {totalTiles} tiles (raw data, fast mode)...");
			yield return null;
			
			// STEP 1: Stitch tiles into pixel buffer - FAST direct memory copy
			byte[] outputPixels;
			var outputWidth = tileData.OutputSize.x;
			var outputHeight = tileData.OutputSize.y;
			var chunkWidth = tileData.ChunkSize.x;
			var chunkHeight = tileData.ChunkSize.y;
			
			try {
				outputPixels = new byte[(long)outputWidth * outputHeight * 4];
			} catch (OutOfMemoryException) {
				Utils.DisplayChatMessage("Out of memory during stitch!");
				yield break;
			}
			
			var lastYieldTime = Time.realtimeSinceStartup;
			var tilesProcessed = 0;
			
			// Pre-calculate row size for fast copy
			var tileRowBytes = chunkWidth * 4;
			
			for (var i = 0; i < tileData.TileImages.Count; i++) {
				var rawTileData = tileData.TileImages[i];
				var tilePos = tileData.TilePositions[i];
				
				// Calculate destination position
				var dstStartX = tilePos.x * chunkWidth;
				var dstStartY = tilePos.y * chunkHeight;
				
				// Direct memory copy - NO decoding, just raw pixel copy row by row
				for (var row = 0; row < chunkHeight && (dstStartY + row) < outputHeight; row++) {
					var srcOffset = row * tileRowBytes;
					var dstOffset = ((dstStartY + row) * outputWidth + dstStartX) * 4;
					var copyBytes = Math.Min(tileRowBytes, (outputWidth - dstStartX) * 4);
					System.Buffer.BlockCopy(rawTileData, srcOffset, outputPixels, dstOffset, copyBytes);
				}
				
				// Free tile data immediately to reduce memory
				tileData.TileImages[i] = null;
				tilesProcessed++;
				
				// Yield every 200 tiles or every 5 seconds (stitching is now very fast)
				if (tilesProcessed % 200 == 0 || Time.realtimeSinceStartup - lastYieldTime > 5f) {
					lastYieldTime = Time.realtimeSinceStartup;
					var progress = (float)tilesProcessed / totalTiles * 100f;
					Utils.DisplayChatMessage($"Background: Stitching... {tilesProcessed}/{totalTiles} ({progress:F0}%)");
					yield return null;
				}
			}
			
			tileData.TileImages.Clear();
			tileData.TilePositions.Clear();
			
			var stitchTime = Time.realtimeSinceStartup - startTime;
			Utils.DisplayChatMessage($"Background: Stitched in {stitchTime:F1}s! Encoding to JPG...");
			yield return null;
			
			// STEP 2: Encode to JPG using pure C# encoder (no dependencies, any size!)
			var encodeStartTime = Time.realtimeSinceStartup;
			var width = tileData.OutputSize.x;
			var height = tileData.OutputSize.y;
			
			byte[] finalJpgData = null;
			string encodeError = null;
			
			int jpgQuality = (int)Unity.Mathematics.math.lerp(100f, 90f, tileData.ResolutionScale / 8f);
			Utils.DisplayChatMessage($"Encoding {width}x{height} image with quality {jpgQuality}...");
			yield return null;
			
			// Use pure C# JPEG encoder - no external dependencies, handles any size!
			try {
				finalJpgData = JpegEncoder.Encode(outputPixels, width, height, jpgQuality);
				outputPixels = null; // Free memory immediately
			} catch (System.Exception ex) {
				encodeError = ex.Message;
			}
			
			// Handle encoder failure (must be outside try/catch for yield)
			if (encodeError != null) {
				Utils.DisplayChatMessage($"Pure encoder failed: {encodeError}");
				yield return null;
				
				// Fallback to Unity encoder if image is small enough
				const int MaxTextureSize = 16384;
				if (width <= MaxTextureSize && height <= MaxTextureSize) {
					Utils.DisplayChatMessage("Falling back to Unity encoder...");
					var finalTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
					finalTexture.LoadRawTextureData(outputPixels);
					finalTexture.Apply();
					outputPixels = null;
					
					finalJpgData = finalTexture.EncodeToJPG(jpgQuality);
					Destroy(finalTexture);
				} else {
					Utils.DisplayChatMessage($"Error: Image too large and encoder failed!");
					yield break;
				}
			}
			
			if (finalJpgData == null || finalJpgData.Length == 0) {
				Utils.DisplayChatMessage("Background: Failed to encode JPG!");
				yield break;
			}
			
			var encodeTime = Time.realtimeSinceStartup - encodeStartTime;
			
			// STEP 3: Save the file
			Utils.WriteCapture(captureName, "jpg", finalJpgData);
			
			var totalTime = Time.realtimeSinceStartup - startTime;
			Utils.DisplayChatMessage($"Background complete! ({totalTime:F1}s total, {finalJpgData.Length / (1024 * 1024)}MB JPG)");
		}

		private IEnumerator StartCaptureRoutine() {
			const float FadeInTime = 0.25f;

			if (API.Server.World != null && CurrentCapture.CanPauseSimulation) {
				var entityManager = API.Server.World.EntityManager;

				_loadAreaEntity = entityManager.CreateEntity();
				entityManager.AddComponentData(_loadAreaEntity, default(LocalTransform));
				entityManager.AddComponentData(_loadAreaEntity, new KeepAreaLoadedCD {
					KeepLoadedRadius = 50,
					StartLoadRadius = 50,
					ImmediateLoadRadius = 50
				});
			}
			
			ApplyCaptureEffectsBeforeFade();
			CaptureProgressUI.Fade(1f, FadeInTime);
			
			yield return new WaitForSeconds(FadeInTime);
			
			ApplyCaptureEffectsAfterFade();
			
			// Use appropriate capture method based on mode
			if (_useBackgroundProcessing && _currentTileDataCallback != null) {
				_currentCaptureRoutine = StartCoroutine(CurrentCapture.GetCoroutineForBackgroundProcessing(_currentTileDataCallback));
			} else {
				_currentCaptureRoutine = StartCoroutine(CurrentCapture.GetCoroutine(_currentCaptureCallback));
			}

			_startCaptureRoutine = null;
		}
		
		private IEnumerator EndCaptureRoutine() {
			const float WaitForLoadAreaTime = 1f;
			const float FadeOutTime = 0.25f;

			if (CurrentCapture.CanPauseSimulation) {
				Manager.camera.cameraMovementStyle = CameraManager.CameraMovementStyle.Instant;
				Manager.camera.manualControlTargetPosition = Manager.main.player.GetEntityPosition();
				StartCoroutine(ResetCameraRoutine());
				
				yield return new WaitForSeconds(WaitForLoadAreaTime);
			}
			
			ClearCaptureEffectsBeforeFade();
			CaptureProgressUI.Fade(0f, FadeOutTime);
			
			yield return new WaitForSeconds(FadeOutTime);
			
			if (CurrentCapture is IDisposable disposable)
				disposable.Dispose();

			CurrentCapture = null;
			_currentCaptureCallback = null;
			_currentCaptureIsComplete = false;

			if (_loadAreaEntity != Entity.Null && API.Server.World != null) {
				var entityManager = API.Server.World.EntityManager;
				entityManager.DestroyEntity(_loadAreaEntity);
			}
			
			_endCaptureRoutine = null;
		}
		
		public void UpdateCapture() {
			if (_currentCaptureIsComplete && _startCaptureRoutine == null && _endCaptureRoutine == null) {
				if (_currentCaptureRoutine != null) {
					StopCoroutine(_currentCaptureRoutine);
					_currentCaptureRoutine = null;
				}
				_endCaptureRoutine = StartCoroutine(EndCaptureRoutine());
			}

			if (!Manager.menu.IsAnyMenuActive() && _queueCaptureStoppedMessage) {
				Utils.DisplayChatMessage(LocalizationManager.GetTranslation("CameraMode:CaptureStopped"));
				_queueCaptureStoppedMessage = false;
			}

			if (_loadAreaEntity != Entity.Null && API.Server.World != null && Manager.main.player != null)
				EntityUtility.UpdatePosition(
					_loadAreaEntity,
					API.Server.World,
					CurrentCapture is FrameCapture && Manager.camera.currentCameraStyle == CameraManager.CameraControlStyle.Static
						? Manager.camera.manualControlTargetPosition
						: Manager.camera.smoothedCameraPosition
				);
		}
		
		public void StopCapture() {
			if (CurrentCapture is { CanPauseSimulation: false } || !IsCapturing || _endCaptureRoutine != null)
				return;

			_currentCaptureIsComplete = true;
			_queueCaptureStoppedMessage = true;
		}
		
		private void ApplyCaptureEffectsBeforeFade() {
			if (CurrentCapture.CanPauseSimulation)
				Manager.input.DisableInput();
			
			_uiWasDisabled = Manager.prefs.hideInGameUI;
			Manager.prefs.hideInGameUI = true;
		}
		
		private void ApplyCaptureEffectsAfterFade() {
			_pauseVisuals = true;
			_startTime = Time.time;
			
			_oldDynamicWaterSetting = Manager.prefs.dynamicWater;
			_simulationWasDisabled = Utils.IsSimulationDisabled;
			
			Manager.prefs.dynamicWater = 0;
			if (Utils.IsSingleplayer && CurrentCapture.CanPauseSimulation && CurrentCapture is not FrameCapture)
				Manager.networking.SetDisableSimulation(true, API.Client.World);
		}

		private void ClearCaptureEffectsBeforeFade() {
			_pauseVisuals = false;
			Manager.prefs.hideInGameUI = _uiWasDisabled;
			Manager.prefs.dynamicWater = _oldDynamicWaterSetting;
			if (Utils.IsSimulationDisabled && CurrentCapture.CanPauseSimulation)
				Manager.networking.SetDisableSimulation(_simulationWasDisabled, API.Client.World);
			
			if (CurrentCapture.CanPauseSimulation)
				Manager.input.EnableInput();
		}
		
		private IEnumerator ResetCameraRoutine() {
			yield return new WaitForEndOfFrame();

			Manager.camera.cameraMovementStyle = CameraManager.CameraMovementStyle.Smooth;
			Manager.camera.currentCameraStyle = CameraManager.CameraControlStyle.FollowPlayer;
		}

		private void UpdatePauseVisuals() {
			foreach (var tracked in _trackedEntityMonos.Values) {
				var entityMono = tracked.EntityMono;

				if (entityMono is Firefly firefly) {
					var particles = firefly.particles;
					var main = particles.main;
					var shouldPause = _pauseVisuals && !tracked.WasRecentlyCreated;

					if (main.simulationSpeed != 0f && shouldPause) {
						particles.Simulate(5f);
						particles.Play();
						main.simulationSpeed = 0f;
					} else if (main.simulationSpeed == 0f && !shouldPause) {
						main.simulationSpeed = 1f;
					}
				}

				if (entityMono is SlimeBoss && entityMono.animator != null && entityMono.XScaler != null && entityMono.XScaler.gameObject.activeSelf)
					entityMono.animator.enabled = !_pauseVisuals;
			}
		}
		
		[HarmonyPatch]
		public static class Patches {
			[HarmonyPatch(typeof(CameraManager), "UpdateGameAndUICameras")]
			[HarmonyPostfix]
			public static void CameraManager_UpdateGameAndUICameras(CameraManager __instance, float deltaTime) {
				if (Instance == null || !Instance._pauseVisuals)
					return;

				API.Rendering.GameCamera.integerScaling = true;
			}

			[HarmonyPatch(typeof(PugRP), "SetupCameraProperties")]
			[HarmonyPostfix]
			public static void PugRP_SetupCameraProperties(PugRPContext context, CommandBuffer cmd, Camera camera, bool forceSkew) {
				if (Instance == null || !Instance._pauseVisuals)
					return;

				var time = Instance._startTime;
				var deltaTime = 0.000001f;
				var smoothDeltaTime = 0.000001f;

				var pTime = time * new Vector4(0.05f, 1f, 2f, 3f);
				var pSinTime = new Vector4(Mathf.Sin(time / 8f), Mathf.Sin(time / 4f), Mathf.Sin(time / 2f), Mathf.Sin(time));
				var pCosTime = new Vector4(Mathf.Cos(time / 8f), Mathf.Cos(time / 4f), Mathf.Cos(time / 2f), Mathf.Cos(time));
				var pDeltaTime = new Vector4(deltaTime, 1f / deltaTime, smoothDeltaTime, 1f / smoothDeltaTime);
				var pTimeParameters = new Vector4(time, Mathf.Sin(time), Mathf.Cos(time), 0f);

				cmd.SetGlobalVector(ShaderIDs.Time, pTime);
				cmd.SetGlobalVector(ShaderIDs.SinTime, pSinTime);
				cmd.SetGlobalVector(ShaderIDs.CosTime, pCosTime);
				cmd.SetGlobalVector(ShaderIDs.DeltaTime, pDeltaTime);
				cmd.SetGlobalVector(ShaderIDs.TimeParameters, pTimeParameters);
				cmd.SetGlobalFloat(_WaterSimDelta, 0.000001f);

				Shader.SetGlobalVector(ShaderIDs.Time, pTime);
				Shader.SetGlobalVector(ShaderIDs.SinTime, pSinTime);
				Shader.SetGlobalVector(ShaderIDs.CosTime, pCosTime);
				Shader.SetGlobalVector(ShaderIDs.DeltaTime, pDeltaTime);
				Shader.SetGlobalVector(ShaderIDs.TimeParameters, pTimeParameters);
				Shader.SetGlobalFloat(_WaterSimDelta, 0.000001f);
			}

			private static readonly MemberInfo MiDeltaTime = typeof(SpriteObject).GetMembersChecked().FirstOrDefault(x => x.GetNameChecked() == "s_deltaTime");

			[HarmonyPatch(typeof(SpriteObject), "GetSpriteArrayCache")]
			[HarmonyPostfix]
			private static void SpriteObject_GetSpriteArrayCache(Dictionary<int, SpriteObject> spriteObjects) {
				if (Instance == null || !Instance._pauseVisuals)
					return;

				API.Reflection.SetValue(MiDeltaTime, null, 0f);
			}

			[HarmonyPatch(typeof(WaterSim), "UpdateSimulation")]
			[HarmonyPrefix]
			private static void WaterSim_UpdateSimulation(WaterSim __instance, ref float deltaTime) {
				if (Instance == null || !Instance._pauseVisuals)
					return;

				deltaTime = 0.000001f;
			}

			private static readonly int _WaterSimDelta = Shader.PropertyToID("_WaterSimDelta");

			[HarmonyPatch(typeof(WaterSim), "LateUpdate")]
			[HarmonyPostfix]
			private static void WaterSim_LateUpdate(WaterSim __instance) {
				if (Instance == null || !Instance._pauseVisuals)
					return;

				Shader.SetGlobalFloat(_WaterSimDelta, 0.000001f);
			}

			[HarmonyPatch(typeof(LightManager), "UpdateLightFlickerEffect")]
			[HarmonyPrefix]
			private static bool LightManager_UpdateLightFlickerEffect(LightManager __instance) {
				if (Instance == null || !Instance._pauseVisuals)
					return true;

				return false;
			}

			[HarmonyPatch(typeof(DroppedItem), "UpdateAnimation")]
			[HarmonyPrefix]
			private static bool DroppedItem_UpdateAnimation(DroppedItem __instance) {
				if (Instance == null || !Instance._pauseVisuals)
					return true;

				return false;
			}

			[HarmonyPatch(typeof(PlayerController), "UpdateBlinking")]
			[HarmonyPrefix]
			private static bool PlayerController_UpdateBlinking(PlayerController __instance) {
				if (Instance == null || !Instance._pauseVisuals)
					return true;

				return false;
			}

			[HarmonyPatch(typeof(CoreBossOrb), "ManagedLateUpdate")]
			[HarmonyPrefix]
			private static bool CoreBossOrb_ManagedLateUpdate(CoreBossOrb __instance) {
				if (Instance == null || !Instance._pauseVisuals)
					return true;

				return false;
			}
		}
	}
}