// A constant buffer that stores the three basic column-major matrices for composing geometry.
cbuffer ModelViewProjectionConstantBuffer : register(b0)
{
	matrix model;
	matrix view;
	matrix projection;
};

// Per-vertex data used as input to the vertex shader.
struct VertexShaderInput
{
	float3 pos : POSITION;
	float4 data0 : COLOR0;
	float4 data1 : COLOR1;
	float4 data2 : COLOR2;
	float4 data3 : COLOR3;
	float4 data4 : COLOR4;
	float4 data5 : COLOR5;
	float data6 : COLOR6;
};

// Per-pixel color data passed through the pixel shader.
struct GeometryShaderInput {
	float4 pos : SV_POSITION;
	float3 modelPosition : POSITION0;
	float4 data0 : COLOR0;
	float4 data1 : COLOR1;
	float4 data2 : COLOR2;
	float4 data3 : COLOR3;
	float4 data4 : COLOR4;
	float4 data5 : COLOR5;
	float data6 : COLOR6;
};

// Simple shader to do vertex processing on the GPU.
GeometryShaderInput main(VertexShaderInput input)
{
	GeometryShaderInput output;
	float4 pos = float4(input.pos, 1.0f);

	// Transform the vertex position into projected space.
	output.modelPosition = input.pos;
	pos = mul(pos, model);
	pos = mul(pos, view);
	pos = mul(pos, projection);
	output.pos = pos;

	// Pass the color through without modification.
	output.data0 = input.data0;
	output.data1 = input.data1;
	output.data2 = input.data2;
	output.data3 = input.data3;
	output.data4 = input.data4;
	output.data5 = input.data5;
	output.data6 = input.data6;

	return output;
}
