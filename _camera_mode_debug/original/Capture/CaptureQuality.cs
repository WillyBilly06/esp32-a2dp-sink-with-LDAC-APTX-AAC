using System;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.Rendering;

namespace CameraMode.Capture {
	public enum CaptureQuality {
		Uncompressed,
		Compressed
	}
	
	public static class CaptureQualityExtensions {
		public static string GetFileExtension(this CaptureQuality quality) {
			return quality switch {
				CaptureQuality.Uncompressed => "png",
				CaptureQuality.Compressed => "jpg",
				_ => throw new ArgumentOutOfRangeException()
			};
		}
		
		public static byte[] EncodeArrayToImage(this CaptureQuality quality, int resolutionScale, byte[] data, GraphicsFormat format, uint width, uint height, uint rowBytes = 0u) {
			return quality switch {
				CaptureQuality.Uncompressed => ImageConversion.EncodeArrayToPNG(data, format, width, height, rowBytes),
				CaptureQuality.Compressed => ImageConversion.EncodeArrayToJPG(data, format, width, height, rowBytes, (int) math.lerp(100f, 25f, resolutionScale / 8f)),
				_ => throw new ArgumentOutOfRangeException()
			};
		}
	}
}