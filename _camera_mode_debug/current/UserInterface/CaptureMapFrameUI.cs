using System.Linq;
using CameraMode.Capture;
using PugMod;
using UnityEngine;

namespace CameraMode.UserInterface {
	public class CaptureMapFrameUI : MonoBehaviour {
		private static readonly MemberInfo MiMapContentMaterial = typeof(MapUI).GetMembersChecked().FirstOrDefault(x => x.GetNameChecked() == "_mapContentMaterial");
		
		public GameObject root;
		public SpriteRenderer frame;
		public SpriteRenderer pinA;
		public SpriteRenderer pinB;

		private void Awake() {
			var mapContentMaterial = (Material) API.Reflection.GetValue(MiMapContentMaterial, Manager.ui.mapUI);
			frame.material = mapContentMaterial;
			pinA.material = mapContentMaterial;
			pinB.material = mapContentMaterial;
		}
		
		private void Update() {
			var captureUI = CaptureManager.Instance.CaptureUI;
			
			root.SetActive(captureUI.IsOpen && Manager.ui.mapUI.IsShowingBigMap && !Manager.prefs.hideInGameUI);
			if (!root.activeSelf)
				return;
			
			CaptureFrameUI.UpdateFrameSr(frame, captureUI.Frame);

			CaptureFrameUI.UpdatePinSr(pinA, captureUI.Frame.PinA);
			CaptureFrameUI.UpdatePinSr(pinB, captureUI.Frame.PinB);
		}
	}
}