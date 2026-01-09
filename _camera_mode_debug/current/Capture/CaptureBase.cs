using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using CameraMode.Utilities;
using PugMod;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using Object = UnityEngine.Object;

namespace CameraMode.Capture {
	// Data class to hold captured tiles for background processing
	public class CapturedTileData {
		public List<byte[]> TileImages;
		public List<int2> TilePositions;
		public int2 ChunkSize;
		public int2 OutputSize;
		public int ResolutionScale;
		public CaptureQuality Quality;
	}
	
	public abstract class CaptureBase {
		public abstract float Progress { get; }
		public virtual float2 DetailedProgress { get; } = float2.zero;
		
		public virtual bool CanPauseSimulation { get; } = false;

		public abstract IEnumerator GetCoroutine(Action<byte[]> callback);
		
		// New method for background processing mode - returns tile data instead of encoded image
		public virtual IEnumerator GetCoroutineForBackgroundProcessing(Action<CapturedTileData> callback) {
			// Default: not supported, fall back to normal mode
			callback?.Invoke(null);
			yield break;
		}
		
		public virtual bool SupportsBackgroundProcessing => false;
	}

	public class ScreenshotCapture : CaptureBase, IDisposable {
		public override float Progress => 1f;

		private Texture2D _captureTexture;
		private RenderTexture _renderTexture;
		
		public override IEnumerator GetCoroutine(Action<byte[]> callback) {
			var captureResScale = Config.Instance.CaptureResolutionScale;
			var captureQuality = Config.Instance.CaptureQuality;
			
			var gameCamera = API.Rendering.GameCamera.camera;

			var outputSize = new int2(
				Mathf.CeilToInt(Constants.kScreenPixelWidth * captureResScale),
				Mathf.CeilToInt(Constants.kScreenPixelHeight * captureResScale)
			);
			var outputPixels = new byte[(outputSize.x * outputSize.y) * 4];
			
			yield return new WaitForEndOfFrame();
			
			_captureTexture = new Texture2D(Constants.kScreenPixelWidth * captureResScale, Constants.kScreenPixelHeight * captureResScale, TextureFormat.RGB24, false);
			_renderTexture = new RenderTexture(Constants.kScreenPixelWidth * captureResScale, Constants.kScreenPixelHeight * captureResScale, 24);
			
			var oldActiveRenderTexture = RenderTexture.active;
			var oldTargetTexture = gameCamera.targetTexture;

			RenderTexture.active = _renderTexture;
			gameCamera.targetTexture = _renderTexture;
			gameCamera.Render();

			_captureTexture.ReadPixels(new Rect(0, 0, _captureTexture.width, _captureTexture.height), 0, 0);
			_captureTexture.Apply();

			Utils.CopyToPixelBuffer(_captureTexture, ref outputPixels, 0, 0, outputSize.x, outputSize.y);

			gameCamera.targetTexture = oldTargetTexture;
			RenderTexture.active = oldActiveRenderTexture;
			
			var encodedImageData = captureQuality.EncodeArrayToImage(captureResScale, outputPixels, GraphicsFormat.R8G8B8A8_SRGB, (uint) outputSize.x, (uint) outputSize.y);
			callback?.Invoke(encodedImageData);
		}

		public void Dispose() {
			if (_captureTexture != null)
				Object.Destroy(_captureTexture);
			if (_renderTexture != null)
				Object.Destroy(_renderTexture);
		}
	}
	
	public class FrameCapture : CaptureBase, IDisposable {
		// Memory limits - increased for high quality captures
		private const long MaxFinalImageBytes = 4L * 1024L * 1024L * 1024L; // 4GB for final image
		private const int MaxFinalDimension = 65536; // Increased max dimension (will be handled by chunked encoding)

		public override float Progress => (float) _areasCaptured / _totalSteps;
		public override float2 DetailedProgress => new(_areasCaptured, _areasToCapture);
		public override bool CanPauseSimulation => true;
		public override bool SupportsBackgroundProcessing => true;

		private readonly CaptureFrame _frame;
		
		private int _areasCaptured;
		private readonly int _areasToCapture;
		private int _totalSteps;

		private Texture2D _captureTexture;
		private RenderTexture _renderTexture;
		
		public FrameCapture(CaptureFrame frame) {
			_frame = frame;

			var frameSize = _frame.Size;
			
			_areasCaptured = 0;
			_areasToCapture = Mathf.CeilToInt((frameSize.x * Constants.PIXELS_PER_UNIT_F) / Constants.kScreenPixelWidth) 
			                * Mathf.CeilToInt((frameSize.y * Constants.PIXELS_PER_UNIT_F) / Constants.kScreenPixelHeight);
			_totalSteps = _areasToCapture + 1; // +1 for stitching phase
		}
		
		// Background processing mode: Only capture tiles, return data for background stitching/encoding
		public override IEnumerator GetCoroutineForBackgroundProcessing(Action<CapturedTileData> callback) {
			Manager.camera.currentCameraStyle = CameraManager.CameraControlStyle.Static;
			Manager.camera.manualControlTargetPosition = Manager.main.player.GetEntityPosition();
			
			var captureResScale = Config.Instance.CaptureResolutionScale;
			var captureQuality = Config.Instance.CaptureQuality;
			var framePosition = _frame.Position;
			var frameSize = _frame.Size;
			
			var gameCamera = API.Rendering.GameCamera.camera;

			var chunks = new int2(
				Mathf.CeilToInt((frameSize.x * Constants.PIXELS_PER_UNIT_F) / Constants.kScreenPixelWidth),
				Mathf.CeilToInt((frameSize.y * Constants.PIXELS_PER_UNIT_F) / Constants.kScreenPixelHeight)
			);
			
			var effectiveResScale = captureResScale;
			var chunkSize = new int2(
				Constants.kScreenPixelWidth * effectiveResScale,
				Constants.kScreenPixelHeight * effectiveResScale
			);
			var outputSize = new int2(chunks.x * chunkSize.x, chunks.y * chunkSize.y);
			
			var screenUnitSize = new float2(
				Constants.kScreenPixelWidth / Constants.PIXELS_PER_UNIT_F,
				Constants.kScreenPixelHeight / Constants.PIXELS_PER_UNIT_F
			);
			
			var finalMemoryMB = (long)outputSize.x * outputSize.y * 4 / (1024 * 1024);
			Utils.DisplayChatMessage($"Capturing at {effectiveResScale}x ({outputSize.x}x{outputSize.y}, ~{finalMemoryMB}MB)");
			
			yield return null;

			var captureSize = new int2(
				Constants.kScreenPixelWidth * effectiveResScale,
				Constants.kScreenPixelHeight * effectiveResScale
			);
			_captureTexture = new Texture2D(captureSize.x, captureSize.y, TextureFormat.RGBA32, false);
			_renderTexture = new RenderTexture(captureSize.x, captureSize.y, 24);

			// Store raw pixel data (no compression) for instant stitching
			var tileImages = new List<byte[]>(_areasToCapture);
			var tilePositions = new List<int2>(_areasToCapture);
			
			var tileMemoryMB = (long)captureSize.x * captureSize.y * 4 * _areasToCapture / (1024 * 1024);
			Utils.DisplayChatMessage($"Capturing {_areasToCapture} tiles (~{tileMemoryMB}MB RAM)...");
			yield return null;
			
			var x = 0;
			var y = 0;
			var direction = 1;

			var areaLoadWaitTime = Config.Instance.AreaLoadWaitTime;
			for (var i = 0; i < chunks.x * chunks.y; i++) {
				Manager.camera.manualControlTargetPosition = new Vector3(
					framePosition.x + (screenUnitSize.x / 2f) - 0.5f + (screenUnitSize.x * x),
					Manager.camera.manualControlTargetPosition.y,
					framePosition.y + (screenUnitSize.y / 2f) - 0.5f + (screenUnitSize.y * y)
				);

				yield return new WaitForSeconds(areaLoadWaitTime);
				Manager.camera.cameraMovementStyle = CameraManager.CameraMovementStyle.Instant;
				yield return new WaitForSeconds(0.05f);
				yield return new WaitForEndOfFrame();
				Manager.camera.cameraMovementStyle = CameraManager.CameraMovementStyle.Smooth;

				var oldActiveRenderTexture = RenderTexture.active;
				var oldTargetTexture = gameCamera.targetTexture;

				RenderTexture.active = _renderTexture;
				gameCamera.targetTexture = _renderTexture;
				gameCamera.Render();

				_captureTexture.ReadPixels(new Rect(0, 0, _captureTexture.width, _captureTexture.height), 0, 0);
				_captureTexture.Apply();

				gameCamera.targetTexture = oldTargetTexture;
				RenderTexture.active = oldActiveRenderTexture;

				// Store RAW pixels - no encoding, instant stitch later
				var rawData = _captureTexture.GetRawTextureData();
				var tileCopy = new byte[rawData.Length];
				System.Buffer.BlockCopy(rawData, 0, tileCopy, 0, rawData.Length);
				tileImages.Add(tileCopy);
				tilePositions.Add(new int2(x, y));

				_areasCaptured++;

				if (_areasCaptured % 30 == 0) {
					yield return null;
				}

				y += direction;
				if (y < 0 || y >= chunks.y) {
					x++;
					y -= direction;
					direction *= -1;
				}
			}
			
			// Free capture textures
			if (_captureTexture != null) {
				Object.Destroy(_captureTexture);
				_captureTexture = null;
			}
			if (_renderTexture != null) {
				Object.Destroy(_renderTexture);
				_renderTexture = null;
			}
			
			_areasCaptured = _totalSteps; // Mark as 100%
			
			// Return tile data for background processing - capture is "done" from user's perspective
			callback?.Invoke(new CapturedTileData {
				TileImages = tileImages,
				TilePositions = tilePositions,
				ChunkSize = chunkSize,
				OutputSize = outputSize,
				ResolutionScale = effectiveResScale,
				Quality = captureQuality
			});
		}

		public override IEnumerator GetCoroutine(Action<byte[]> callback) {
			Manager.camera.currentCameraStyle = CameraManager.CameraControlStyle.Static;
			Manager.camera.manualControlTargetPosition = Manager.main.player.GetEntityPosition();
			
			var captureResScale = Config.Instance.CaptureResolutionScale;
			var captureQuality = Config.Instance.CaptureQuality;
			var framePosition = _frame.Position;
			var frameSize = _frame.Size;
			
			var gameCamera = API.Rendering.GameCamera.camera;

			var chunks = new int2(
				Mathf.CeilToInt((frameSize.x * Constants.PIXELS_PER_UNIT_F) / Constants.kScreenPixelWidth),
				Mathf.CeilToInt((frameSize.y * Constants.PIXELS_PER_UNIT_F) / Constants.kScreenPixelHeight)
			);
			
			// Always use the user-selected resolution scale - no auto-downscaling
			var effectiveResScale = captureResScale;
			var chunkSize = new int2(
				Constants.kScreenPixelWidth * effectiveResScale,
				Constants.kScreenPixelHeight * effectiveResScale
			);
			var outputSize = new int2(chunks.x * chunkSize.x, chunks.y * chunkSize.y);
			
			// No downsampling - always capture at full selected resolution
			var downsampleFactor = 1;
			
			var screenUnitSize = new float2(
				Constants.kScreenPixelWidth / Constants.PIXELS_PER_UNIT_F,
				Constants.kScreenPixelHeight / Constants.PIXELS_PER_UNIT_F
			);
			
			var finalMemoryMB = (long)outputSize.x * outputSize.y * 4 / (1024 * 1024);
			Utils.DisplayChatMessage($"Capturing at {effectiveResScale}x resolution ({outputSize.x}x{outputSize.y}, ~{finalMemoryMB}MB)");
			
			yield return null;

			// Create capture texture at full resolution (for quality), will downsample if needed
			var captureSize = new int2(
				Constants.kScreenPixelWidth * effectiveResScale,
				Constants.kScreenPixelHeight * effectiveResScale
			);
			_captureTexture = new Texture2D(captureSize.x, captureSize.y, TextureFormat.RGB24, false);
			_renderTexture = new RenderTexture(captureSize.x, captureSize.y, 24);

			// Store all tiles as compressed JPG (much smaller than raw)
			var tileImages = new List<byte[]>(_areasToCapture);
			var tilePositions = new List<int2>(_areasToCapture);
			
			Utils.DisplayChatMessage($"Capturing {_areasToCapture} tiles...");
			yield return null;
			
			var x = 0;
			var y = 0;
			var direction = 1;

			// PHASE 1: Capture all tiles as compressed images
			var areaLoadWaitTime = Config.Instance.AreaLoadWaitTime;
			for (var i = 0; i < chunks.x * chunks.y; i++) {
				Manager.camera.manualControlTargetPosition = new Vector3(
					framePosition.x + (screenUnitSize.x / 2f) - 0.5f + (screenUnitSize.x * x),
					Manager.camera.manualControlTargetPosition.y,
					framePosition.y + (screenUnitSize.y / 2f) - 0.5f + (screenUnitSize.y * y)
				);

				yield return new WaitForSeconds(areaLoadWaitTime);
				Manager.camera.cameraMovementStyle = CameraManager.CameraMovementStyle.Instant;
				yield return new WaitForSeconds(0.05f);
				yield return new WaitForEndOfFrame();
				Manager.camera.cameraMovementStyle = CameraManager.CameraMovementStyle.Smooth;

				var oldActiveRenderTexture = RenderTexture.active;
				var oldTargetTexture = gameCamera.targetTexture;

				RenderTexture.active = _renderTexture;
				gameCamera.targetTexture = _renderTexture;
				gameCamera.Render();

				_captureTexture.ReadPixels(new Rect(0, 0, _captureTexture.width, _captureTexture.height), 0, 0);
				_captureTexture.Apply();

				gameCamera.targetTexture = oldTargetTexture;
				RenderTexture.active = oldActiveRenderTexture;

				// Encode tile as JPG (high quality, ~10x smaller than raw)
				var tileData = _captureTexture.EncodeToJPG(95);
				tileImages.Add(tileData);
				tilePositions.Add(new int2(x, y));

				_areasCaptured++;

				// Yield every 30 tiles to keep UI responsive (removed GC.Collect for speed)
				if (_areasCaptured % 30 == 0) {
					yield return null;
				}

				y += direction;
				if (y < 0 || y >= chunks.y) {
					x++;
					y -= direction;
					direction *= -1;
				}
			}
			
			// Free capture textures
			if (_captureTexture != null) {
				Object.Destroy(_captureTexture);
				_captureTexture = null;
			}
			if (_renderTexture != null) {
				Object.Destroy(_renderTexture);
				_renderTexture = null;
			}
			yield return null;

			// PHASE 2: Stitch tiles into final image
			Utils.DisplayChatMessage($"Stitching {_areasToCapture} tiles into {outputSize.x}x{outputSize.y} image...");
			yield return null;
			
			// Allocate final pixel buffer
			byte[] outputPixels;
			try {
				outputPixels = new byte[(long)outputSize.x * outputSize.y * 4];
			} catch (OutOfMemoryException) {
				Utils.DisplayChatMessage("Out of memory during stitch! Area too large.");
				callback?.Invoke(null);
				yield break;
			}
			
			// Temp texture for decoding tiles
			var tempTexture = new Texture2D(2, 2, TextureFormat.RGB24, false);
			var stitchStartTime = Time.realtimeSinceStartup;
			
			for (var i = 0; i < tileImages.Count; i++) {
				var tileData = tileImages[i];
				var tilePos = tilePositions[i];
				
				// Decode JPG back to texture
				tempTexture.LoadImage(tileData);
				
				// If we need to downsample, resize the texture
				if (downsampleFactor > 1 || tempTexture.width != chunkSize.x || tempTexture.height != chunkSize.y) {
					var resizedTexture = ResizeTexture(tempTexture, chunkSize.x, chunkSize.y);
					if (resizedTexture != tempTexture) {
						Object.Destroy(tempTexture);
						tempTexture = resizedTexture;
					}
				}
				
				// Copy to output buffer
				Utils.CopyToPixelBuffer(tempTexture, ref outputPixels, tilePos.x * chunkSize.x, tilePos.y * chunkSize.y, outputSize.x, outputSize.y);
				
				// Free the tile data immediately
				tileImages[i] = null;
				
				// Yield less frequently for speed (every 50 tiles or every 2 seconds)
				if (i % 50 == 0 || Time.realtimeSinceStartup - stitchStartTime > 2f) {
					stitchStartTime = Time.realtimeSinceStartup;
					Utils.DisplayChatMessage($"Stitching... {i + 1}/{tileImages.Count}");
					yield return null;
				}
			}
			
			Object.Destroy(tempTexture);
			tileImages.Clear();
			tilePositions.Clear();
			
			_areasCaptured = _totalSteps; // Stitching complete
			
			// PHASE 3: Encode final image
			var encodingStartTime = Time.realtimeSinceStartup;
			var imageSizeGB = (long)outputSize.x * outputSize.y * 4 / (1024.0 * 1024.0 * 1024.0);
			Utils.DisplayChatMessage($"Encoding {outputSize.x}x{outputSize.y} image ({imageSizeGB:F2}GB raw)...");
			yield return null;
			
			// Encode the image
			var encodedImageData = captureQuality.EncodeArrayToImage(effectiveResScale, outputPixels, GraphicsFormat.R8G8B8A8_SRGB, (uint)outputSize.x, (uint)outputSize.y);
			
			var encodingTime = Time.realtimeSinceStartup - encodingStartTime;
			Utils.DisplayChatMessage($"Encoding completed in {encodingTime:F1}s ({encodedImageData.Length / (1024 * 1024)}MB)");
			
			outputPixels = null;
			
			callback?.Invoke(encodedImageData);
		}
		
		private Texture2D ResizeTexture(Texture2D source, int targetWidth, int targetHeight) {
			var rt = RenderTexture.GetTemporary(targetWidth, targetHeight, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
			rt.filterMode = FilterMode.Bilinear;
			
			RenderTexture.active = rt;
			Graphics.Blit(source, rt);
			
			var result = new Texture2D(targetWidth, targetHeight, TextureFormat.RGB24, false);
			result.ReadPixels(new Rect(0, 0, targetWidth, targetHeight), 0, 0);
			result.Apply();
			
			RenderTexture.active = null;
			RenderTexture.ReleaseTemporary(rt);
			
			return result;
		}

		public void Dispose() {
			if (_captureTexture != null)
				Object.Destroy(_captureTexture);
			if (_renderTexture != null)
				Object.Destroy(_renderTexture);
		}
	}
}