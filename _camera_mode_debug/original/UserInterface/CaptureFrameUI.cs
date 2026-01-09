using CameraMode.Capture;
using Pug.UnityExtensions;
using Unity.Mathematics;
using UnityEngine;

namespace CameraMode.UserInterface {
	public class CaptureFrameUI : MonoBehaviour {
		 public GameObject root;
		 public SpriteRenderer frame;
		 public SpriteRenderer pinA;
		 public SpriteRenderer pinB;
		 public SpriteRenderer pinPreview;
		
		private void Update() {
			var captureUI = CaptureManager.Instance.CaptureUI;
			
			root.SetActive(captureUI.IsOpen && !Manager.ui.mapUI.IsShowingBigMap && !Manager.prefs.hideInGameUI);
			if (!root.activeSelf)
				return;
			
			UpdateFrameSr(frame, captureUI.Frame);

			UpdatePinSr(pinA, captureUI.Frame.PinA);
			UpdatePinSr(pinB, captureUI.Frame.PinB);
			UpdatePinSr(pinPreview, captureUI.PinPreview);
		}

		public static void UpdateFrameSr(SpriteRenderer sr, CaptureFrame frame) {
			if (frame.IsComplete) {
				sr.size = new Vector2(frame.Size.x, frame.Size.y);
				sr.transform.localPosition = new Vector3(frame.Center.x, frame.Center.y, sr.transform.localPosition.z);
			} else {
				sr.size = new Vector2(0f, 0f);
			}
		}

		public static void UpdatePinSr(SpriteRenderer sr, int2? tilePosition) {
			if (tilePosition.HasValue) {
				var position = tilePosition.Value.ToFloat2() + 0.5f;
				sr.transform.localPosition = new Vector3(position.x, position.y, sr.transform.localPosition.z);
				sr.color = sr.color.ColorWithNewAlpha(1f);
			} else {
				sr.color = sr.color.ColorWithNewAlpha(0f);
			}
		}
	}
}