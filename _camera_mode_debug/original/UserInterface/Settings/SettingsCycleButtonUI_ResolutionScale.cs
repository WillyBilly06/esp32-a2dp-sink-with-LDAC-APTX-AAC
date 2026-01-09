using System.Collections.Generic;

namespace CameraMode.UserInterface.Settings {
	// ReSharper disable once InconsistentNaming
	public class SettingsCycleButtonUI_ResolutionScale : SettingsCycleButtonUI<int> {
		protected override List<int> Values => new() {
			1, 2, 4, 8
		};
		protected override int DefaultValue => 2;
		
		protected override int GetConfigValue() {
			return Config.Instance.CaptureResolutionScale;
		}

		protected override void SetConfigValue(int value) {
			Config.Instance.CaptureResolutionScale = value;
		}

		protected override void UpdateText() {
			valueText.Render($"{GetConfigValue()}x");
		}
	}
}