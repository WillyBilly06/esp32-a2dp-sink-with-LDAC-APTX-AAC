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
			CaptureQuality.Compressed
		};
		private static readonly List<int> CaptureResolutionScaleValues = new() {
			1, 2, 4, 8
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
				_ => throw new ArgumentOutOfRangeException()
			});
		}

		public override void OnLeftClicked(bool mod1, bool mod2) {
			base.OnLeftClicked(mod1, mod2);
			
			CycleSelectedValue(1);
		}

		private void CycleSelectedValue(int offset) {
			SetSelectedValue(settingType switch {
				SettingType.CaptureQuality => (CaptureQualityValues.IndexOf(Config.Instance.CaptureQuality) + offset) % CaptureQualityValues.Count,
				SettingType.CaptureResolutionScale => (CaptureResolutionScaleValues.IndexOf(Config.Instance.CaptureResolutionScale) + offset) % CaptureResolutionScaleValues.Count,
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
				default:
					throw new ArgumentOutOfRangeException();
			}
		}
		
		public enum SettingType {
			CaptureQuality,
			CaptureResolutionScale
		}
	}
}