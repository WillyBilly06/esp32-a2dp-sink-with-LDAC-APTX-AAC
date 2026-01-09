using System;
using System.IO;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.Rendering;

namespace CameraMode.Capture {
	public enum CaptureQuality {
		Uncompressed,    // PNG - slow but lossless
		Compressed,      // JPG (80-95%) - fast, slight quality loss
		FastUncompressed // Fast mode - capture tiles quickly, stitch+encode in background
	}
	
	public static class CaptureQualityExtensions {
		// Extension for the output file
		public static string GetFileExtension(this CaptureQuality quality) {
			return quality switch {
				CaptureQuality.Uncompressed => "png",
				CaptureQuality.Compressed => "jpg",
				CaptureQuality.FastUncompressed => "jpg", // Fast mode outputs JPG
				_ => throw new ArgumentOutOfRangeException()
			};
		}
		
		// Returns true if this quality mode uses background processing
		public static bool UsesBackgroundConversion(this CaptureQuality quality) {
			return quality == CaptureQuality.FastUncompressed;
		}
		
		public static byte[] EncodeArrayToImage(this CaptureQuality quality, int resolutionScale, byte[] data, GraphicsFormat format, uint width, uint height, uint rowBytes = 0u) {
			return quality switch {
				// PNG is slow but lossless
				CaptureQuality.Uncompressed => ImageConversion.EncodeArrayToPNG(data, format, width, height, rowBytes),
				// JPG with quality based on resolution (80-95%)
				CaptureQuality.Compressed => ImageConversion.EncodeArrayToJPG(data, format, width, height, rowBytes, (int) math.lerp(95f, 80f, resolutionScale / 8f)),
				// Fast mode uses direct JPG encoding (handled separately via background processing)
				CaptureQuality.FastUncompressed => ImageConversion.EncodeArrayToJPG(data, format, width, height, rowBytes, 95),
				_ => throw new ArgumentOutOfRangeException()
			};
		}
	}
}