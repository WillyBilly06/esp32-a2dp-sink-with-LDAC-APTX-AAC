using System;
using System.Collections.Generic;
using CameraMode.Capture;
using Pug.UnityExtensions;
using UnityEngine;

namespace CameraMode.UserInterface.Options {
	public class CaptureSettingsButtonUI : ButtonUIElement {
		private static readonly Color SelectedTextColor = new(0.647f, 0.792f, 0.855f, 1f);
		private static readonly Color UnselectedTextColor = new(1f, 1f, 1f, 0.35f);
		
		public SettingType settingType;
		public PugText labelText;
		public PugText valueText;

		private bool _wasActive;
		
		private static readonly List<CaptureQuality> CaptureQualityValues = new() {
			CaptureQuality.Uncompressed,
			CaptureQuality.Compressed,
			CaptureQuality.FastUncompressed
		};
		private static readonly List<int> CaptureResolutionScaleValues = new() {
			1, 2, 4, 8
		};
		private static readonly List<float> AreaLoadWaitTimeValues = new() {
			0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f,
			1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f, 1.7f, 1.8f, 1.9f, 2.0f
		};

		public override void OnSelected() {
			base.OnSelected();
			
			labelText.SetTempColor(SelectedTextColor);
			valueText.SetTempColor(SelectedTextColor);
		}

		public override void OnDeselected(bool playEffect = true) {
			base.OnDeselected(playEffect);
			
			labelText.SetTempColor(UnselectedTextColor);
			valueText.SetTempColor(UnselectedTextColor);
		}

		private void OnValidate() {
			labelText.style.color = UnselectedTextColor;
			valueText.style.color = UnselectedTextColor;
		}
		
		protected override void LateUpdate() {
			base.LateUpdate();
			
			SetSelectedValue(settingType switch {
				SettingType.CaptureQuality => CaptureQualityValues.IndexOf(Config.Instance.CaptureQuality),
				SettingType.CaptureResolutionScale => CaptureResolutionScaleValues.IndexOf(Config.Instance.CaptureResolutionScale),
				SettingType.AreaLoadWaitTime => GetClosestWaitTimeIndex(Config.Instance.AreaLoadWaitTime),
				_ => throw new ArgumentOutOfRangeException()
			});
		}
		
		private static int GetClosestWaitTimeIndex(float value) {
			var closestIndex = 0;
			var closestDiff = float.MaxValue;
			for (var i = 0; i < AreaLoadWaitTimeValues.Count; i++) {
				var diff = Math.Abs(AreaLoadWaitTimeValues[i] - value);
				if (diff < closestDiff) {
					closestDiff = diff;
					closestIndex = i;
				}
			}
			return closestIndex;
		}

		public override void OnLeftClicked(bool mod1, bool mod2) {
			base.OnLeftClicked(mod1, mod2);
			
			CycleSelectedValue(1);
		}

		private void CycleSelectedValue(int offset) {
			SetSelectedValue(settingType switch {
				SettingType.CaptureQuality => (CaptureQualityValues.IndexOf(Config.Instance.CaptureQuality) + offset) % CaptureQualityValues.Count,
				SettingType.CaptureResolutionScale => (CaptureResolutionScaleValues.IndexOf(Config.Instance.CaptureResolutionScale) + offset) % CaptureResolutionScaleValues.Count,
				SettingType.AreaLoadWaitTime => (GetClosestWaitTimeIndex(Config.Instance.AreaLoadWaitTime) + offset) % AreaLoadWaitTimeValues.Count,
				_ => throw new ArgumentOutOfRangeException()
			});
		}

		private void SetSelectedValue(int index) {
			switch (settingType) {
				case SettingType.CaptureQuality:
					Config.Instance.CaptureQuality = CaptureQualityValues.IsValidIndex(index) ? CaptureQualityValues[index] : CaptureQualityValues[0];
					valueText.Render($"CameraMode:CaptureQuality/{Config.Instance.CaptureQuality}");
					break;
				case SettingType.CaptureResolutionScale:
					Config.Instance.CaptureResolutionScale = CaptureResolutionScaleValues.IsValidIndex(index) ? CaptureResolutionScaleValues[index] : CaptureResolutionScaleValues[0];
					valueText.Render($"{Config.Instance.CaptureResolutionScale}x");
					break;
				case SettingType.AreaLoadWaitTime:
					Config.Instance.AreaLoadWaitTime = AreaLoadWaitTimeValues.IsValidIndex(index) ? AreaLoadWaitTimeValues[index] : AreaLoadWaitTimeValues[9];
					valueText.Render($"{Config.Instance.AreaLoadWaitTime:F1}s");
					break;
				default:
					throw new ArgumentOutOfRangeException();
			}
		}
		
		public enum SettingType {
			CaptureQuality,
			CaptureResolutionScale,
			AreaLoadWaitTime
		}
	}
}