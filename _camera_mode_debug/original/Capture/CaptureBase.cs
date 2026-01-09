using System;
using System.Collections;
using CameraMode.Utilities;
using PugMod;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using Object = UnityEngine.Object;

namespace CameraMode.Capture {
	public abstract class CaptureBase {
		public abstract float Progress { get; }
		public virtual float2 DetailedProgress { get; } = float2.zero;
		
		public virtual bool CanPauseSimulation { get; } = false;

		public abstract IEnumerator GetCoroutine(Action<byte[]> callback);
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
		private const float AreaLoadWaitTime = 1.5f;

		public override float Progress => (float) _areasCaptured / _areasToCapture;
		public override float2 DetailedProgress => new(_areasCaptured, _areasToCapture);
		public override bool CanPauseSimulation => true;

		private readonly CaptureFrame _frame;
		
		private int _areasCaptured;
		private readonly int _areasToCapture;

		private Texture2D _captureTexture;
		private RenderTexture _renderTexture;
		
		public FrameCapture(CaptureFrame frame) {
			_frame = frame;

			var frameSize = _frame.Size;
			
			_areasCaptured = 0;
			_areasToCapture = Mathf.CeilToInt((frameSize.x * Constants.PIXELS_PER_UNIT_F) / Constants.kScreenPixelWidth) * Mathf.CeilToInt((frameSize.y * Constants.PIXELS_PER_UNIT_F) / Constants.kScreenPixelHeight);
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
			var chunkSize = new int2(
				Constants.kScreenPixelWidth * captureResScale,
				Constants.kScreenPixelHeight * captureResScale
			);
			var screenUnitSize = new float2(
				Constants.kScreenPixelWidth / Constants.PIXELS_PER_UNIT_F,
				Constants.kScreenPixelHeight / Constants.PIXELS_PER_UNIT_F
			);
			var outputSize = new int2(
				Mathf.CeilToInt(frameSize.x * (Constants.PIXELS_PER_UNIT_F * (chunkSize.x / (float) Constants.kScreenPixelWidth))),
				Mathf.CeilToInt(frameSize.y * (Constants.PIXELS_PER_UNIT_F * (chunkSize.y / (float) Constants.kScreenPixelHeight)))
			);
			var outputPixels = new byte[(outputSize.x * outputSize.y) * 4];

			_captureTexture = new Texture2D(Constants.kScreenPixelWidth * captureResScale, Constants.kScreenPixelHeight * captureResScale, TextureFormat.RGB24, false);
			_renderTexture = new RenderTexture(Constants.kScreenPixelWidth * captureResScale, Constants.kScreenPixelHeight * captureResScale, 24);
			
			var x = 0;
			var y = 0;
			var direction = 1;

			for (var i = 0; i < chunks.x * chunks.y; i++) {
				Manager.camera.manualControlTargetPosition = new Vector3(
					framePosition.x + (screenUnitSize.x / 2f) - 0.5f + (screenUnitSize.x * x),
					Manager.camera.manualControlTargetPosition.y,
					framePosition.y + (screenUnitSize.y / 2f) - 0.5f + (screenUnitSize.y * y)
				);

				yield return new WaitForSeconds(AreaLoadWaitTime);
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

				Utils.CopyToPixelBuffer(_captureTexture, ref outputPixels, x * chunkSize.x, y * chunkSize.y, outputSize.x, outputSize.y);

				gameCamera.targetTexture = oldTargetTexture;
				RenderTexture.active = oldActiveRenderTexture;

				_areasCaptured++;

				y += direction;
				if (y < 0 || y >= chunks.y) {
					x++;
					y -= direction;
					direction *= -1;
				}
			}
			
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
}