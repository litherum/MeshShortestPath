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
	float3 data1 : COLOR1;
};

// Per-pixel color data passed through the pixel shader.
struct GeometryShaderInput {
	float4 pos : SV_POSITION;
	float3 modelPosition : POSITION0;
	float4 data0 : COLOR0;
	float3 data1 : COLOR1;
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

	return output;
}
