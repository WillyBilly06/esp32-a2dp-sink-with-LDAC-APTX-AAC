using System.Collections.Generic;

namespace CameraMode.UserInterface.Settings {
	// ReSharper disable once InconsistentNaming
	public class SettingsCycleButtonUI_WaitTime : SettingsCycleButtonUI<float> {
		protected override List<float> Values => new() {
			0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 
			1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f, 1.7f, 1.8f, 1.9f, 2.0f
		};
		protected override float DefaultValue => 1.0f;
		
		protected override float GetConfigValue() {
			return Config.Instance.AreaLoadWaitTime;
		}

		protected override void SetConfigValue(float value) {
			Config.Instance.AreaLoadWaitTime = value;
		}

		protected override void UpdateText() {
			valueText.Render($"{GetConfigValue():F1}s");
		}
	}
}
