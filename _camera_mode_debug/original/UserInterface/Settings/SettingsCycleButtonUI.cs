using System.Collections.Generic;
using Pug.UnityExtensions;
using UnityEngine;

namespace CameraMode.UserInterface.Settings {
	public abstract class SettingsCycleButtonUI<T> : ButtonUIElement {
		private static readonly Color SelectedTextColor = new(0.647f, 0.792f, 0.855f, 1f);
		private static readonly Color UnselectedTextColor = new(1f, 1f, 1f, 0.35f);
		
		public PugText labelText;
		public PugText valueText;

		private bool _wasActive;
		
		protected abstract List<T> Values { get; }
		protected abstract T DefaultValue { get; }
		
		public override void OnSelected() {
			base.OnSelected();
			
			UpdateTextColor(SelectedTextColor);
		}

		public override void OnDeselected(bool playEffect = true) {
			base.OnDeselected(playEffect);
			
			UpdateTextColor(UnselectedTextColor);
		}

		private void OnValidate() {
			if (labelText == null || valueText == null)
				return;
			
			UpdateTextColor(UnselectedTextColor);
		}
		
		protected override void LateUpdate() {
			base.LateUpdate();

			var isActive = gameObject.activeInHierarchy;
			if (isActive && !_wasActive)
				SetSelectedValue(Values.IndexOf(GetConfigValue()));

			_wasActive = isActive;
		}

		public override void OnLeftClicked(bool mod1, bool mod2) {
			base.OnLeftClicked(mod1, mod2);
			
			CycleSelectedValue(1);
		}

		private void CycleSelectedValue(int offset) {
			SetSelectedValue((Values.IndexOf(GetConfigValue()) + offset) % Values.Count);
		}

		private void SetSelectedValue(int index) {
			SetConfigValue(Values.IsValidIndex(index) ? Values[index] : DefaultValue);
			UpdateText();
		}

		protected abstract T GetConfigValue();
		
		protected abstract void SetConfigValue(T value);

		protected abstract void UpdateText();

		private void UpdateTextColor(Color color) {
			labelText.style.color = color;
			labelText.SetTempColor(color);
			valueText.style.color = color;
			valueText.SetTempColor(color);
		}
	}
}