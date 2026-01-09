using System.Collections.Generic;
using CameraMode.Capture;

namespace CameraMode.UserInterface.Settings {
	// ReSharper disable once InconsistentNaming
	public class SettingsCycleButtonUI_Quality : SettingsCycleButtonUI<CaptureQuality> {
		protected override List<CaptureQuality> Values => new() {
			CaptureQuality.Uncompressed,
			CaptureQuality.Compressed,
			CaptureQuality.FastUncompressed
		};
		protected override CaptureQuality DefaultValue => CaptureQuality.Uncompressed;
		
		protected override CaptureQuality GetConfigValue() {
			return Config.Instance.CaptureQuality;
		}

		protected override void SetConfigValue(CaptureQuality value) {
			Config.Instance.CaptureQuality = value;
		}

		protected override void UpdateText() {
			valueText.Render($"CameraMode:CaptureQuality/{GetConfigValue()}");
		}
	}
}