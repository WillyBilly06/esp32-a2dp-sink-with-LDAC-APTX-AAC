using CameraMode.Capture;
using CameraMode.Utilities;
using HarmonyLib;
using Pug.UnityExtensions;
using PugMod;
using Unity.Mathematics;
using UnityEngine;

// ReSharper disable InconsistentNaming

namespace CameraMode.UserInterface {
	public class CaptureUI : MonoBehaviour {
		public GameObject buttonContainer;
		public GameObject settingsContainer;
		public PugText inCameraModeText;
		public GameObject captureFramePrefab;
		public GameObject captureMapFramePrefab;
		public CaptureButtonUI captureFrameButton;
		public CaptureButtonUI clearFrameButton;
		public CaptureButtonUI pinFrameButton;
		public CaptureButtonUI toggleMapButton;
		public CaptureButtonUI settingsButton;

		public bool IsOpen { get; private set; }
		public CaptureFrame Frame { get; private set; }
		public Mode SelectedMode { get; private set; }
		
		public int2? PinPreview { get; private set; }
		
		private Transform _renderAnchor;
		
		private void Start() {
			Frame = new CaptureFrame();
			_renderAnchor = Manager.camera.GetRenderAnchor();
			
			Instantiate(captureFramePrefab, _renderAnchor);
			Instantiate(captureMapFramePrefab, Manager.ui.mapUI.mapPartsContainer.transform);
			
			API.Client.OnWorldCreated += () => {
				Frame = new CaptureFrame();
			};
		}

		private void OnDestroy() {
			if (_renderAnchor != null)
				Manager.camera.ReturnRenderAnchor(_renderAnchor);
		}
		
		private void Update() {
			var isCapturing = CaptureManager.Instance.IsCapturing;
			
			buttonContainer.transform.localScale = Manager.ui.CalcGameplayUITargetScaleMultiplier();
			inCameraModeText.transform.localScale = isCapturing ? Vector3.one : buttonContainer.transform.localScale;
			
			buttonContainer.SetActive(IsOpen);
			inCameraModeText.gameObject.SetActive(IsOpen || isCapturing);
			inCameraModeText.SetTempColor(inCameraModeText.color.ColorWithNewAlpha(IsOpen ? 1f : CaptureManager.Instance.CaptureProgressUI.Opacity));
			
			if (!buttonContainer.activeSelf)
				return;
			
			settingsContainer.SetActive(SelectedMode == Mode.Settings);
			
			var input = Manager.input.singleplayerInputModule;
			PinPreview = null;
			
			if (SelectedMode == Mode.PinFrame && input.PrefersKeyboardAndMouse() && !Manager.ui.mapUI.IsShowingBigMap && Manager.ui.currentSelectedUIElement == null) {
				var mouseTilePosition = EntityMonoBehaviour.ToWorldFromRender(Manager.ui.mouse.GetMouseGameViewPosition()).RoundToInt2();
				PinPreview = mouseTilePosition;
				
				if (input.IsButtonCurrentlyDown(PlayerInput.InputType.UI_INTERACT))
					Frame.PinA = mouseTilePosition;

				if (input.IsButtonCurrentlyDown(PlayerInput.InputType.UI_SECOND_INTERACT))
					Frame.PinB = mouseTilePosition;
			}
			
			if (Frame.PinA == null && Frame.PinB == null)
				clearFrameButton.SetDisabled(CaptureButtonUI.DisabledReason.NoFrameSet);
			else
				clearFrameButton.SetEnabled();
			
			if (!Frame.IsComplete)
				captureFrameButton.SetDisabled(CaptureButtonUI.DisabledReason.NoFrameSet);
			else
				captureFrameButton.SetEnabled();
			
			pinFrameButton.isOn = SelectedMode == Mode.PinFrame;
			settingsButton.isOn = SelectedMode == Mode.Settings;
			toggleMapButton.isOn = Manager.ui.mapUI.IsShowingBigMap;
		}
		
		public void Open() {
			IsOpen = true;
		}

		public void Close() {
			IsOpen = false;
			SelectedMode = Mode.None;
		}

		public void PinFrame() {
			SelectedMode = SelectedMode == Mode.PinFrame ? Mode.None : Mode.PinFrame;
		}
		
		public void ResetFrame() {
			Frame = new CaptureFrame();
		}

		public void ToggleMap() {
			Manager.ui.OnMapToggle();
		}

		public void TakeScreenshotCapture() {
			CaptureManager.Instance.Capture(new ScreenshotCapture());
		}

		public void TakeFrameCapture() {
			if (!Frame.IsComplete)
				return;
			
			CaptureManager.Instance.Capture(new FrameCapture(Frame));
		}
		
		public void OpenFolder() {
			Utils.OpenCaptureDirectory();
		}
		
		public void ToggleSettings() {
			SelectedMode = SelectedMode == Mode.Settings ? Mode.None : Mode.Settings;
		}

		public enum Mode {
			None,
			PinFrame,
			Settings
		}
		
		[HarmonyPatch]
		private static class Patches {
			[HarmonyPatch(typeof(PlayerController), "get_guestMode")]
			[HarmonyPostfix]
			private static void PlayerController_get_guestMode(PlayerController __instance, ref bool __result) {
				if (CaptureManager.Instance.CaptureUI.IsOpen)
					__result = true;
			}
			
			[HarmonyPatch(typeof(StatusText), "LateUpdate")]
			[HarmonyPostfix]
			private static void StatusText_LateUpdate(StatusText __instance) {
				if (CaptureManager.Instance.CaptureUI.IsOpen)
					__instance.container.SetActive(false);
			}
			
			[HarmonyPatch(typeof(SendClientInputSystem), "PlayerInteractionBlocked")]
			[HarmonyPostfix]
			private static void SendClientInputSystem_PlayerInteractionBlocked(SendClientInputSystem __instance, ref bool __result) {
				if (!CaptureManager.Instance.CaptureUI.IsOpen && Time.time <= CaptureButtonUI.LastPressedTime + 0.1f)
					__result = true;
			}
			
			[HarmonyPatch(typeof(MapUI), "UpdateMapFromUserInput")]
			[HarmonyPrefix]
			private static bool MapUI_UpdateMapFromUserInput(MapUI __instance) {
				var captureUI = CaptureManager.Instance.CaptureUI;
				if (!captureUI.IsOpen || captureUI.SelectedMode != Mode.PinFrame || !__instance.IsShowingBigMap || __instance.OpenedMapThisFrame || Time.time <= CaptureButtonUI.LastPressedTime + 0.1f)
					return true;
				
				var input = Manager.input.singleplayerInputModule;
				var cursorWorldPosition = __instance.GetCursorWorldPosition().RoundToInt2();
				
				if (input.IsButtonCurrentlyDown(PlayerInput.InputType.UI_INTERACT))
					captureUI.Frame.PinA = cursorWorldPosition;

				if (input.IsButtonCurrentlyDown(PlayerInput.InputType.UI_SECOND_INTERACT))
					captureUI.Frame.PinB = cursorWorldPosition;

				return false;
			}
		}
	}
}