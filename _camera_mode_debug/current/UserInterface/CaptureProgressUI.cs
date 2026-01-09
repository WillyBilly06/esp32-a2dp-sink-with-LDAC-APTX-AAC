using CameraMode.Capture;
using Pug.UnityExtensions;
using Unity.Mathematics;
using UnityEngine;

namespace CameraMode.UserInterface {
	public class CaptureProgressUI : UIelement {
		public GameObject root;
		public GameObject textContainer;
		public SpriteRenderer background;
		public PugText progressText;
		public PugText areasCapturedText;
		
		public float Opacity { get; private set; }

		private float _fadeTarget;
		private TimerSimple _fadeTimer;

		public void Fade(float target, float duration) {
			_fadeTarget = target;
			_fadeTimer = TimerSimple.StartNew(duration);
		}
		
		protected override void LateUpdate() {
			var captureManager = CaptureManager.Instance;
			var currentCapture = captureManager.CurrentCapture;
			
			Opacity = math.lerp(Opacity, _fadeTarget, math.clamp(_fadeTimer.elapsedRatio, 0f, 1f));

			if (Opacity > 0f) {
				root.SetActive(true);
				textContainer.SetActive(!Manager.menu.IsAnyMenuActive() && currentCapture?.DetailedProgress.y > 0f);
				
				if (currentCapture != null) {
					progressText.Render($"{math.ceil(currentCapture.Progress * 100f)}%");
					areasCapturedText.Render($"({currentCapture.DetailedProgress.x}/{currentCapture.DetailedProgress.y})");
				}
				
				background.color = background.color.ColorWithNewAlpha(Opacity);
				progressText.SetTempColor(progressText.color.ColorWithNewAlpha(Opacity));
				areasCapturedText.SetTempColor(progressText.color.ColorWithNewAlpha(Opacity));
			} else {
				root.SetActive(false);
			}
		}
	}
}