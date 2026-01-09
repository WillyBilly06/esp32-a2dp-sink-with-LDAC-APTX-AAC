using System;
using CameraMode.Capture;
using PugMod;
using Unity.Mathematics;

namespace CameraMode {
	public class Config {
		public static Config Instance { get; private set; } = new();
		
		private int _captureResolutionScale;
		private IConfigEntry<int> _captureResolutionScaleEntry;
		public int CaptureResolutionScale {
			get => _captureResolutionScale;
			set {
				_captureResolutionScale = math.clamp(value, 1, 8);
				_captureResolutionScaleEntry.Value = _captureResolutionScale;
			}
		}
		
		private CaptureQuality _captureQuality;
		private IConfigEntry<CaptureQuality> _captureQualityEntry;
		public CaptureQuality CaptureQuality {
			get => _captureQuality;
			set {
				_captureQuality = Enum.IsDefined(typeof(CaptureQuality), value) ? value : CaptureQuality.Uncompressed;
				_captureQualityEntry.Value = _captureQuality;
			}
		}
		
		public void Init() {
			_captureResolutionScaleEntry = API.Config.Register(Main.InternalName, "Capture", "", "ResolutionScale", 2);
			CaptureResolutionScale = _captureResolutionScaleEntry.Value;
			
			_captureQualityEntry = API.Config.Register(Main.InternalName, "Capture", "", "Quality", CaptureQuality.Uncompressed);
			CaptureQuality = _captureQualityEntry.Value;
		}
	}
}