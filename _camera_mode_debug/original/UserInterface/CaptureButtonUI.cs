using System.Collections.Generic;
using CameraMode.Capture;
using UnityEngine;

namespace CameraMode.UserInterface {
	public class CaptureButtonUI : ToggleUIElement {
		public static float LastPressedTime { get; private set; }
		
		public bool canBeToggled;
		public SpriteRenderer icon;
		public SpriteRenderer background;
		public Color onBackgroundColor;
		public Color offBackgroundColor;
		public Color canBeClickedIconColor;
		public Color cantBeClickedIconColor;

		private bool _isSelected;
		private DisabledReason _disabledReason;

		public void SetEnabled() {
			canBeClicked = true;
			_disabledReason = DisabledReason.None;
		}
		
		public void SetDisabled(DisabledReason disabledReason = DisabledReason.None) {
			canBeClicked = false;
			_disabledReason = disabledReason;
		}
		
		public override void OnLeftClicked(bool mod1, bool mod2) {
			if (CaptureManager.Instance.IsCapturing)
				return;
			
			base.OnLeftClicked(mod1, mod2);

			if (!canBeToggled && isOn)
				isOn = false;

			LastPressedTime = Time.time;
		}

		protected override void LateUpdate() {
			base.LateUpdate();

			var isSelectedOrOn = canBeClicked && (_isSelected || (canBeToggled && isOn));

			icon.color = canBeClicked ? canBeClickedIconColor : cantBeClickedIconColor;
			background.color = isSelectedOrOn ? onBackgroundColor : offBackgroundColor;
		}

		public override void OnSelected() {
			_isSelected = true;
		}

		public override void OnDeselected(bool playEffect = true) {
			_isSelected = false;
		}

		public override HoverWindowAlignment GetHoverWindowAlignment() {
			return HoverWindowAlignment.BOTTOM_RIGHT_OF_CURSOR;
		}
		
		public override List<TextAndFormatFields> GetHoverDescription() {
			if (showHoverDesc) {
				var lines = new List<TextAndFormatFields> {
					new() {
						text = optionalHoverDesc.mTerm,
						color = Manager.text.GetRarityColor(Rarity.Poor)
					}
				};
				
				if (!canBeClicked && _disabledReason != DisabledReason.None) {
					lines.Add(new TextAndFormatFields {
						text = "CameraMode:DisabledReason/" + _disabledReason,
						color = Manager.ui.brokenColor
					});
				}

				return lines;
			}
			return base.GetHoverDescription();
		}

		public void OnValidate() {
			background.color = onBackgroundColor;
		}

		public enum DisabledReason {
			None,
			NoFrameSet
		}
	}
}