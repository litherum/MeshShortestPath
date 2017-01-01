struct GeometryShaderInput {
	float4 pos : SV_POSITION;
	float3 worldPosition : POSITION0;
	float4 data0 : COLOR0;
	float3 data1 : COLOR1;
};

struct PixelShaderInput {
	float4 pos : SV_POSITION;
	float3 worldPosition : POSITION0;
	nointerpolation float3 vertexWorldPositions[3] : POSITION1;
	nointerpolation float4 data0[3] : COLOR0;
	nointerpolation float3 data1[3] : COLOR3;
};

[maxvertexcount(3)]
void main(triangle GeometryShaderInput input[3], inout TriangleStream<PixelShaderInput> output) {
	float3 vertexWorldPositions[3];
	float4 data0[3];
	float3 data1[3];

	for (uint i = 0; i < 3; ++i) {
		vertexWorldPositions[i] = input[i].worldPosition;
		data0[i] = input[i].data0;
		data1[i] = input[i].data1;
	}

	for (uint i = 0; i < 3; i++) {
		PixelShaderInput element;
		element.pos = input[i].pos;
		element.worldPosition = input[i].worldPosition;
		element.vertexWorldPositions = vertexWorldPositions;
		element.data0 = data0;
		element.data1 = data1;
		output.Append(element);
	}
}
