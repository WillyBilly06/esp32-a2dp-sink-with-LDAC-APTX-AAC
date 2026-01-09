using Unity.Mathematics;
using UnityEngine;

namespace CameraMode.Capture {
	public class CaptureFrame {
		public int2? PinA;
		public int2? PinB;
		
		public bool IsComplete => PinA != null && PinB != null && Size.x > 0 && Size.y > 0;

		public Vector2 Position {
			get {
				if (PinA == null || PinB == null)
					return default;
				
				var minX = math.min(PinA.Value.x, PinB.Value.x);
				var minY = math.min(PinA.Value.y, PinB.Value.y);
				
				return new Vector2(minX, minY);
			}
		}
		public Vector2 Size {
			get {
				if (PinA == null || PinB == null)
					return default;
				
				var minX = math.min(PinA.Value.x, PinB.Value.x);
				var minY = math.min(PinA.Value.y, PinB.Value.y);
				var maxX = math.max(PinA.Value.x, PinB.Value.x) + 1;
				var maxY = math.max(PinA.Value.y, PinB.Value.y) + 1;

				return new Vector2(math.abs(maxX - minX), math.abs(maxY - minY));
			}
		}

		public Vector2 Center => Position + (Size / 2f);
	}
}