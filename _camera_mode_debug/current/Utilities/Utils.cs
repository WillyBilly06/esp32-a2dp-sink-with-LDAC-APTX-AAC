using System;
using System.Linq;
using I2.Loc;
using PugMod;
using UnityEngine;

namespace CameraMode.Utilities {
	public static class Utils {
		private const string CaptureDirectoryName = Main.InternalName + "/Captures";

		private static readonly MemberInfo MiRenderText = typeof(ChatWindow).GetMembersChecked().FirstOrDefault(x => x.GetNameChecked() == "RenderText");
		private static readonly MemberInfo MiPlatformInterface = typeof(Manager).GetMembersChecked().FirstOrDefault(x => x.GetNameChecked() == "_platformInterface");
		
		public static bool IsSingleplayer => Manager.ecs.ServerWorld != null && Manager.ecs.ServerConnectionQ.CalculateEntityCount() <= 1;
		public static bool IsSimulationDisabled => API.Client.World.GetExistingSystemManaged<WorldInfoSystem>().WorldInfo.simulationDisabled;

		public static void OpenCaptureDirectory() {
			var modConfigPath = GetModConfigDirectory();
			if (modConfigPath == null)
				return;

			TryCreateCaptureDirectory();
			
			var capturesPath = modConfigPath + "/" + CaptureDirectoryName;
			Application.OpenURL("file://" + capturesPath);
		}

		private static string GetModConfigDirectory() {
			var platformInterface = (PlatformInterface) API.Reflection.GetValue(MiPlatformInterface, Manager.main);
			if (platformInterface == null)
				return null;
			
			var persistentPath = Application.persistentDataPath;
			if (!string.IsNullOrEmpty(platformInterface.SavePrefix))
				persistentPath = persistentPath + "/" + platformInterface.SavePrefix;

			persistentPath = persistentPath + "/" + platformInterface.Name + "/" + platformInterface.GetAccountId();
			persistentPath = persistentPath.Replace('\\', '/');
			
			return persistentPath + "/mods";
		}

		private static void TryCreateCaptureDirectory() {
			if (!API.ConfigFilesystem.DirectoryExists(CaptureDirectoryName))
				API.ConfigFilesystem.CreateDirectory(CaptureDirectoryName);
		}
		
		public static void WriteCapture(string baseName, string extension, byte[] data) {
			// Data is null when capture was cancelled or failed
			if (data == null || data.Length == 0)
				return;
			
			TryCreateCaptureDirectory();

			extension ??= "png";
			
			var name = $"{baseName}.{extension}";
			var path = $"{CaptureDirectoryName}/{name}";

			var index = 1;
			while (API.ConfigFilesystem.FileExists(path)) {
				name = $"{baseName} ({index}).{extension}";
				path = $"{CaptureDirectoryName}/{name}";
				index++;
			}
			
			API.ConfigFilesystem.Write(path, data);
			
			DisplayChatMessage(string.Format(LocalizationManager.GetTranslation("CameraMode:SavedCapture"), name));
		}
		
		public static void DeleteCapture(string baseName, string extension) {
			var name = $"{baseName}.{extension}";
			var path = $"{CaptureDirectoryName}/{name}";
			
			if (API.ConfigFilesystem.FileExists(path)) {
				try {
					API.ConfigFilesystem.Delete(path);
				} catch {
					// Ignore deletion errors
				}
			}
		}
		
		public static void WriteCaptureText(string baseName, string extension, string text) {
			TryCreateCaptureDirectory();
			
			var name = $"{baseName}.{extension}";
			var path = $"{CaptureDirectoryName}/{name}";
			
			var data = System.Text.Encoding.UTF8.GetBytes(text);
			API.ConfigFilesystem.Write(path, data);
		}

		public static void DisplayChatMessage(string text) {
			if (text == null)
				return;
			
			API.Reflection.Invoke(MiRenderText, Manager.ui.chatWindow, text);
		}
		
				public static void CopyToPixelBuffer(Texture2D sourceTexture, ref byte[] targetPixels, int startX, int startY, int targetWidth, int targetHeight) {
			if (sourceTexture == null || targetPixels == null)
				return;

			var sourceWidth = sourceTexture.width;
			var sourceHeight = sourceTexture.height;

			// Clamp copy region to destination bounds
			var copyWidth = Mathf.Min(sourceWidth, targetWidth - startX);
			var copyHeight = Mathf.Min(sourceHeight, targetHeight - startY);
			if (copyWidth <= 0 || copyHeight <= 0)
				return;

			// Fast path: copy raw texture bytes row-by-row (no float conversions, minimal allocations).
			// This is expected to be used with RGBA32 capture textures (4 bytes per pixel).
			try {
				var raw = sourceTexture.GetRawTextureData(); // managed byte[] in Unity; should be available without Unity.Collections
				var bytesPerPixel = raw.Length / (sourceWidth * sourceHeight);
				if (bytesPerPixel == 4) {
					var srcStride = sourceWidth * 4;
					for (var y = 0; y < copyHeight; y++) {
						var srcOffset = y * srcStride;
						var dstOffset = ((startY + y) * targetWidth + startX) * 4;
						System.Buffer.BlockCopy(raw, srcOffset, targetPixels, dstOffset, copyWidth * 4);

						// Ensure alpha is fully opaque (PNG path uses alpha; some pipelines can leave it at 0).
						for (var x = 0; x < copyWidth; x++)
							targetPixels[dstOffset + x * 4 + 3] = 255;
					}
					return;
				}
			} catch {
				// Fall through to safe (slower) path below.
			}

			// Safe fallback path (slower, allocates): convert Color[] to bytes.
			var sourcePixels = sourceTexture.GetPixels();
			for (var y = 0; y < sourceHeight; y++) {
				for (var x = 0; x < sourceWidth; x++) {
					var dstX = startX + x;
					var dstY = startY + y;
					if (dstX < 0 || dstY < 0 || dstX >= targetWidth || dstY >= targetHeight)
						continue;

					var targetIndex = (dstY * targetWidth + dstX) * 4;
					var sourceIndex = y * sourceWidth + x;
					targetPixels[targetIndex] = (byte)(sourcePixels[sourceIndex].r * 255);
					targetPixels[targetIndex + 1] = (byte)(sourcePixels[sourceIndex].g * 255);
					targetPixels[targetIndex + 2] = (byte)(sourcePixels[sourceIndex].b * 255);
					targetPixels[targetIndex + 3] = 255;
				}
			}
		}
	}
}