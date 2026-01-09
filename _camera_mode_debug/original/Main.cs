using System.Linq;
using CameraMode.Capture;
using PugMod;
using UnityEngine;
using Object = UnityEngine.Object;
// ReSharper disable InconsistentNaming

namespace CameraMode {
	public class Main : IMod {
		public const string Version = "1.0.5";
		public const string InternalName = "CameraMode";
		public const string DisplayName = "Camera Mode";
		
		internal static AssetBundle AssetBundle { get; private set; }

		public void EarlyInit() {
			var modInfo = API.ModLoader.LoadedMods.FirstOrDefault(modInfo => modInfo.Handlers.Contains(this));
			Debug.Log($"[{DisplayName}] Mod version: {Version}");
			
			AssetBundle = modInfo!.AssetBundles[0];
		}

		public void Init() {
			var gameObject = new GameObject("Camera Mode Managers");
			Object.DontDestroyOnLoad(gameObject);
			gameObject.AddComponent<CaptureManager>();
			
			Config.Instance.Init();
		}

		public void Update() { }
		
		public void Shutdown() { }

		public void ModObjectLoaded(Object obj) { }
	}
}