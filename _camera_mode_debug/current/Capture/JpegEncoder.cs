using System;
using System.Collections.Generic;

namespace CameraMode.Capture {
	/// <summary>
	/// Pure C# JPEG encoder - no external dependencies, no System.IO.
	/// Handles any image size without Unity texture limits.
	/// </summary>
	public static class JpegEncoder {
		// Standard JPEG quantization tables (quality-adjusted at runtime)
		private static readonly byte[] LuminanceQuantTable = {
			16, 11, 10, 16, 24, 40, 51, 61,
			12, 12, 14, 19, 26, 58, 60, 55,
			14, 13, 16, 24, 40, 57, 69, 56,
			14, 17, 22, 29, 51, 87, 80, 62,
			18, 22, 37, 56, 68, 109, 103, 77,
			24, 35, 55, 64, 81, 104, 113, 92,
			49, 64, 78, 87, 103, 121, 120, 101,
			72, 92, 95, 98, 112, 100, 103, 99
		};

		private static readonly byte[] ChrominanceQuantTable = {
			17, 18, 24, 47, 99, 99, 99, 99,
			18, 21, 26, 66, 99, 99, 99, 99,
			24, 26, 56, 99, 99, 99, 99, 99,
			47, 66, 99, 99, 99, 99, 99, 99,
			99, 99, 99, 99, 99, 99, 99, 99,
			99, 99, 99, 99, 99, 99, 99, 99,
			99, 99, 99, 99, 99, 99, 99, 99,
			99, 99, 99, 99, 99, 99, 99, 99
		};

		// Zigzag order for DCT coefficients
		private static readonly int[] ZigZag = {
			0,  1,  8, 16,  9,  2,  3, 10,
			17, 24, 32, 25, 18, 11,  4,  5,
			12, 19, 26, 33, 40, 48, 41, 34,
			27, 20, 13,  6,  7, 14, 21, 28,
			35, 42, 49, 56, 57, 50, 43, 36,
			29, 22, 15, 23, 30, 37, 44, 51,
			58, 59, 52, 45, 38, 31, 39, 46,
			53, 60, 61, 54, 47, 55, 62, 63
		};

		// Standard Huffman tables for DC and AC coefficients
		private static readonly byte[] DC_Luminance_Bits = { 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
		private static readonly byte[] DC_Luminance_Values = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

		private static readonly byte[] DC_Chrominance_Bits = { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };
		private static readonly byte[] DC_Chrominance_Values = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

		private static readonly byte[] AC_Luminance_Bits = { 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125 };
		private static readonly byte[] AC_Luminance_Values = {
			0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
			0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
			0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
			0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
			0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
			0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
			0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
			0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
			0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
			0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
			0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
			0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
			0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
			0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
			0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
			0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
			0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
			0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
			0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
			0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
			0xf9, 0xfa
		};

		private static readonly byte[] AC_Chrominance_Bits = { 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 119 };
		private static readonly byte[] AC_Chrominance_Values = {
			0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
			0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
			0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
			0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
			0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
			0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
			0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
			0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
			0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
			0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
			0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
			0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
			0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
			0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
			0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
			0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
			0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
			0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
			0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
			0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
			0xf9, 0xfa
		};

		// Precomputed cosine values for DCT
		private static readonly float[] CosineTable = new float[64];
		private static readonly float C1 = 1.0f / (float)Math.Sqrt(2.0);

		static JpegEncoder() {
			// Precompute cosine table for DCT
			for (int i = 0; i < 8; i++) {
				for (int j = 0; j < 8; j++) {
					CosineTable[i * 8 + j] = (float)Math.Cos((2 * i + 1) * j * Math.PI / 16.0);
				}
			}
		}

		/// <summary>
		/// Encode RGBA32 pixel data to JPEG format.
		/// </summary>
		/// <param name="pixels">RGBA32 pixel data (4 bytes per pixel)</param>
		/// <param name="width">Image width in pixels</param>
		/// <param name="height">Image height in pixels</param>
		/// <param name="quality">JPEG quality (1-100)</param>
		/// <returns>JPEG file data</returns>
		public static byte[] Encode(byte[] pixels, int width, int height, int quality = 90) {
			// Clamp quality
			quality = Math.Max(1, Math.Min(100, quality));

			// Calculate quantization scale factor
			int scale = quality < 50 ? (5000 / quality) : (200 - quality * 2);

			// Create scaled quantization tables
			byte[] lumQuant = ScaleQuantTable(LuminanceQuantTable, scale);
			byte[] chrQuant = ScaleQuantTable(ChrominanceQuantTable, scale);

			// Build Huffman encoding tables
			var dcLumTable = BuildHuffmanTable(DC_Luminance_Bits, DC_Luminance_Values);
			var acLumTable = BuildHuffmanTable(AC_Luminance_Bits, AC_Luminance_Values);
			var dcChrTable = BuildHuffmanTable(DC_Chrominance_Bits, DC_Chrominance_Values);
			var acChrTable = BuildHuffmanTable(AC_Chrominance_Bits, AC_Chrominance_Values);

			// Use List<byte> instead of MemoryStream (System.IO is blocked)
			var output = new List<byte>(width * height); // Preallocate approximate size
			var writer = new BitWriter(output);

			// Write JPEG headers
			WriteSOI(output);
			WriteAPP0(output);
			WriteDQT(output, lumQuant, 0);
			WriteDQT(output, chrQuant, 1);
			WriteSOF0(output, width, height);
			WriteDHT(output, DC_Luminance_Bits, DC_Luminance_Values, 0, false);
			WriteDHT(output, AC_Luminance_Bits, AC_Luminance_Values, 0, true);
			WriteDHT(output, DC_Chrominance_Bits, DC_Chrominance_Values, 1, false);
			WriteDHT(output, AC_Chrominance_Bits, AC_Chrominance_Values, 1, true);
			WriteSOS(output);

			// Process image in 8x8 blocks
			int prevDcY = 0, prevDcCb = 0, prevDcCr = 0;
			float[] block = new float[64];
			int[] quantized = new int[64];

			int paddedWidth = ((width + 7) / 8) * 8;
			int paddedHeight = ((height + 7) / 8) * 8;

			for (int by = 0; by < paddedHeight; by += 8) {
				for (int bx = 0; bx < paddedWidth; bx += 8) {
					// Extract and encode Y block
					ExtractYBlock(pixels, width, height, bx, by, block);
					ForwardDCT(block);
					Quantize(block, lumQuant, quantized);
					prevDcY = EncodeBlock(writer, quantized, prevDcY, dcLumTable, acLumTable);

					// Extract and encode Cb block
					ExtractCbBlock(pixels, width, height, bx, by, block);
					ForwardDCT(block);
					Quantize(block, chrQuant, quantized);
					prevDcCb = EncodeBlock(writer, quantized, prevDcCb, dcChrTable, acChrTable);

					// Extract and encode Cr block
					ExtractCrBlock(pixels, width, height, bx, by, block);
					ForwardDCT(block);
					Quantize(block, chrQuant, quantized);
					prevDcCr = EncodeBlock(writer, quantized, prevDcCr, dcChrTable, acChrTable);
				}
			}

			writer.Flush();
			WriteEOI(output);

			return output.ToArray();
		}

		private static byte[] ScaleQuantTable(byte[] table, int scale) {
			byte[] result = new byte[64];
			for (int i = 0; i < 64; i++) {
				int val = (table[i] * scale + 50) / 100;
				result[i] = (byte)Math.Max(1, Math.Min(255, val));
			}
			return result;
		}

		private static void ExtractYBlock(byte[] pixels, int width, int height, int bx, int by, float[] block) {
			for (int y = 0; y < 8; y++) {
				int py = Math.Min(by + y, height - 1);
				for (int x = 0; x < 8; x++) {
					int px = Math.Min(bx + x, width - 1);
					int idx = (py * width + px) * 4;
					float r = pixels[idx];
					float g = pixels[idx + 1];
					float b = pixels[idx + 2];
					// RGB to Y conversion, centered at 0
					block[y * 8 + x] = 0.299f * r + 0.587f * g + 0.114f * b - 128f;
				}
			}
		}

		private static void ExtractCbBlock(byte[] pixels, int width, int height, int bx, int by, float[] block) {
			for (int y = 0; y < 8; y++) {
				int py = Math.Min(by + y, height - 1);
				for (int x = 0; x < 8; x++) {
					int px = Math.Min(bx + x, width - 1);
					int idx = (py * width + px) * 4;
					float r = pixels[idx];
					float g = pixels[idx + 1];
					float b = pixels[idx + 2];
					// RGB to Cb conversion
					block[y * 8 + x] = -0.1687f * r - 0.3313f * g + 0.5f * b;
				}
			}
		}

		private static void ExtractCrBlock(byte[] pixels, int width, int height, int bx, int by, float[] block) {
			for (int y = 0; y < 8; y++) {
				int py = Math.Min(by + y, height - 1);
				for (int x = 0; x < 8; x++) {
					int px = Math.Min(bx + x, width - 1);
					int idx = (py * width + px) * 4;
					float r = pixels[idx];
					float g = pixels[idx + 1];
					float b = pixels[idx + 2];
					// RGB to Cr conversion
					block[y * 8 + x] = 0.5f * r - 0.4187f * g - 0.0813f * b;
				}
			}
		}

		private static void ForwardDCT(float[] block) {
			float[] temp = new float[64];

			// Row-wise DCT
			for (int y = 0; y < 8; y++) {
				for (int u = 0; u < 8; u++) {
					float sum = 0;
					for (int x = 0; x < 8; x++) {
						sum += block[y * 8 + x] * CosineTable[x * 8 + u];
					}
					temp[y * 8 + u] = sum * (u == 0 ? C1 : 1.0f);
				}
			}

			// Column-wise DCT
			for (int u = 0; u < 8; u++) {
				for (int v = 0; v < 8; v++) {
					float sum = 0;
					for (int y = 0; y < 8; y++) {
						sum += temp[y * 8 + u] * CosineTable[y * 8 + v];
					}
					block[v * 8 + u] = sum * (v == 0 ? C1 : 1.0f) * 0.25f;
				}
			}
		}

		private static void Quantize(float[] block, byte[] quantTable, int[] output) {
			for (int i = 0; i < 64; i++) {
				int zigzagIdx = ZigZag[i];
				output[i] = (int)Math.Round(block[zigzagIdx] / quantTable[zigzagIdx]);
			}
		}

		private static int EncodeBlock(BitWriter writer, int[] block, int prevDc, 
			(ushort code, int bits)[] dcTable, (ushort code, int bits)[] acTable) {
			
			// Encode DC coefficient (difference from previous block)
			int dcDiff = block[0] - prevDc;
			int dcCategory = GetCategory(dcDiff);
			writer.WriteBits(dcTable[dcCategory].code, dcTable[dcCategory].bits);
			if (dcCategory > 0) {
				int dcValue = dcDiff < 0 ? dcDiff - 1 : dcDiff;
				writer.WriteBits((ushort)(dcValue & ((1 << dcCategory) - 1)), dcCategory);
			}

			// Encode AC coefficients
			int zeroCount = 0;
			for (int i = 1; i < 64; i++) {
				if (block[i] == 0) {
					zeroCount++;
				} else {
					while (zeroCount >= 16) {
						writer.WriteBits(acTable[0xF0].code, acTable[0xF0].bits); // ZRL
						zeroCount -= 16;
					}
					int acCategory = GetCategory(block[i]);
					int symbol = (zeroCount << 4) | acCategory;
					writer.WriteBits(acTable[symbol].code, acTable[symbol].bits);
					int acValue = block[i] < 0 ? block[i] - 1 : block[i];
					writer.WriteBits((ushort)(acValue & ((1 << acCategory) - 1)), acCategory);
					zeroCount = 0;
				}
			}

			if (zeroCount > 0) {
				writer.WriteBits(acTable[0].code, acTable[0].bits); // EOB
			}

			return block[0];
		}

		private static int GetCategory(int value) {
			if (value < 0) value = -value;
			int cat = 0;
			while (value > 0) {
				value >>= 1;
				cat++;
			}
			return cat;
		}

		private static (ushort code, int bits)[] BuildHuffmanTable(byte[] bits, byte[] values) {
			var table = new (ushort code, int bits)[256];
			int code = 0;
			int valueIdx = 0;

			for (int len = 1; len <= 16; len++) {
				for (int i = 0; i < bits[len - 1]; i++) {
					table[values[valueIdx]] = ((ushort)code, len);
					valueIdx++;
					code++;
				}
				code <<= 1;
			}

			return table;
		}

		#region JPEG File Structure Writers (using List<byte> instead of Stream)

		private static void WriteSOI(List<byte> s) {
			s.Add(0xFF); s.Add(0xD8); // Start of Image
		}

		private static void WriteEOI(List<byte> s) {
			s.Add(0xFF); s.Add(0xD9); // End of Image
		}

		private static void WriteAPP0(List<byte> s) {
			s.Add(0xFF); s.Add(0xE0); // APP0 marker
			WriteWord(s, 16); // Length
			s.Add(0x4A); s.Add(0x46); s.Add(0x49); s.Add(0x46); s.Add(0x00); // "JFIF\0"
			s.Add(1); s.Add(1); // Version 1.1
			s.Add(0); // Aspect ratio units (0 = no units)
			WriteWord(s, 1); // X density
			WriteWord(s, 1); // Y density
			s.Add(0); s.Add(0); // No thumbnail
		}

		private static void WriteDQT(List<byte> s, byte[] table, int tableId) {
			s.Add(0xFF); s.Add(0xDB); // DQT marker
			WriteWord(s, 67); // Length
			s.Add((byte)tableId); // Table ID (0 = lum, 1 = chrom)
			for (int i = 0; i < 64; i++) s.Add(table[i]);
		}

		private static void WriteSOF0(List<byte> s, int width, int height) {
			s.Add(0xFF); s.Add(0xC0); // SOF0 marker (baseline DCT)
			WriteWord(s, 17); // Length
			s.Add(8); // Precision (8 bits)
			WriteWord(s, height);
			WriteWord(s, width);
			s.Add(3); // Number of components (Y, Cb, Cr)
			
			// Y component: ID=1, sampling=1x1, quant table=0
			s.Add(1); s.Add(0x11); s.Add(0);
			// Cb component: ID=2, sampling=1x1, quant table=1
			s.Add(2); s.Add(0x11); s.Add(1);
			// Cr component: ID=3, sampling=1x1, quant table=1
			s.Add(3); s.Add(0x11); s.Add(1);
		}

		private static void WriteDHT(List<byte> s, byte[] bits, byte[] values, int tableId, bool isAC) {
			s.Add(0xFF); s.Add(0xC4); // DHT marker
			int length = 19 + values.Length;
			WriteWord(s, length);
			s.Add((byte)((isAC ? 0x10 : 0x00) | tableId));
			for (int i = 0; i < 16; i++) s.Add(bits[i]);
			for (int i = 0; i < values.Length; i++) s.Add(values[i]);
		}

		private static void WriteSOS(List<byte> s) {
			s.Add(0xFF); s.Add(0xDA); // SOS marker
			WriteWord(s, 12); // Length
			s.Add(3); // Number of components
			
			// Y: DC table 0, AC table 0
			s.Add(1); s.Add(0x00);
			// Cb: DC table 1, AC table 1
			s.Add(2); s.Add(0x11);
			// Cr: DC table 1, AC table 1
			s.Add(3); s.Add(0x11);
			
			s.Add(0); s.Add(63); s.Add(0); // Spectral selection
		}

		private static void WriteWord(List<byte> s, int value) {
			s.Add((byte)(value >> 8));
			s.Add((byte)(value & 0xFF));
		}

		#endregion

		/// <summary>
		/// Bit writer with JPEG byte stuffing (0xFF -> 0xFF 0x00)
		/// Uses List<byte> instead of Stream to avoid System.IO
		/// </summary>
		private class BitWriter {
			private List<byte> output;
			private int buffer;
			private int bitsInBuffer;

			public BitWriter(List<byte> outputList) {
				output = outputList;
				buffer = 0;
				bitsInBuffer = 0;
			}

			public void WriteBits(ushort value, int numBits) {
				buffer = (buffer << numBits) | (value & ((1 << numBits) - 1));
				bitsInBuffer += numBits;

				while (bitsInBuffer >= 8) {
					bitsInBuffer -= 8;
					byte b = (byte)((buffer >> bitsInBuffer) & 0xFF);
					output.Add(b);
					if (b == 0xFF) {
						output.Add(0x00); // Byte stuffing
					}
				}
			}

			public void Flush() {
				if (bitsInBuffer > 0) {
					// Pad with 1s
					buffer = (buffer << (8 - bitsInBuffer)) | ((1 << (8 - bitsInBuffer)) - 1);
					byte b = (byte)(buffer & 0xFF);
					output.Add(b);
					if (b == 0xFF) {
						output.Add(0x00);
					}
				}
			}
		}
	}
}
