#pragma once

namespace Viewer
{
	// Constant buffer used to send MVP matrices to the vertex shader.
	struct ModelViewProjectionConstantBuffer
	{
		DirectX::XMFLOAT4X4 model;
		DirectX::XMFLOAT4X4 view;
		DirectX::XMFLOAT4X4 projection;
	};

	// Used to send per-vertex data to the vertex shader.
	struct VertexPositionData
	{
		DirectX::XMFLOAT3 pos;
		DirectX::XMFLOAT4 data0;
		DirectX::XMFLOAT4 data1;
		DirectX::XMFLOAT4 data2;
		DirectX::XMFLOAT4 data3;
		DirectX::XMFLOAT4 data4;
		DirectX::XMFLOAT4 data5;
		float data6;
	};
}