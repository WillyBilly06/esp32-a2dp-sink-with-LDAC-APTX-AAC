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
			} else if (CaptureUI.IsOpen) {
				CaptureUI.Close();
			}
			
			UpdateCapture();
			UpdatePauseVisuals();
		}

		public void Capture(CaptureBase capture, string captureName = null) {
			if (IsCapturing)
				return;
			
			captureName ??= $"Capture {DateTime.Now:yyyy-MM-dd HH.mm.ss}";
			
			CurrentCapture = capture;
			_currentCaptureCallback = imageData => {
				Utils.WriteCapture(captureName, Config.Instance.CaptureQuality.GetFileExtension(), imageData);
				_currentCaptureIsComplete = true;
			};
			_currentCaptureRoutine = null;
			_currentCaptureIsComplete = false;

			_startCaptureRoutine = StartCoroutine(StartCaptureRoutine());
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
			_currentCaptureRoutine = StartCoroutine(CurrentCapture.GetCoroutine(_currentCaptureCallback));

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
				EntityUtility.UpdatePosition(_loadAreaEntity, API.Server.World, Manager.camera.smoothedCameraPosition);
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
			if (Utils.IsSingleplayer && CurrentCapture.CanPauseSimulation)
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