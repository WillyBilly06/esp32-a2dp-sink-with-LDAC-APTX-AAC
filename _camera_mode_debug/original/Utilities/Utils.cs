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

		public static void DisplayChatMessage(string text) {
			if (text == null)
				return;
			
			API.Reflection.Invoke(MiRenderText, Manager.ui.chatWindow, text);
		}
		
		public static void CopyToPixelBuffer(Texture2D sourceTexture, ref byte[] targetPixels, int startX, int startY, int targetWidth, int targetHeight) {
			var sourcePixels = sourceTexture.GetPixels();
			var sourceWidth = sourceTexture.width;
			var sourceHeight = sourceTexture.height;

			for (var y = 0; y < sourceHeight; y++) {
				for (var x = 0; x < sourceWidth; x++) {
					var targetIndex = ((startY + y) * targetWidth + (startX + x)) * 4;
					var sourceIndex = y * sourceWidth + x;

					if (startX + x < targetWidth && startY + y < targetHeight) {
						targetPixels[targetIndex] = (byte) (sourcePixels[sourceIndex].r * 255);
						targetPixels[targetIndex + 1] = (byte) (sourcePixels[sourceIndex].g * 255);
						targetPixels[targetIndex + 2] = (byte) (sourcePixels[sourceIndex].b * 255);
						targetPixels[targetIndex + 3] = (byte) (sourcePixels[sourceIndex].a * 255);
					}
				}
			}
		}
	}
}