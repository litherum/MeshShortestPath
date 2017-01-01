struct GeometryShaderInput {
	float4 pos : SV_POSITION;
	float3 worldPosition : POSITION0;
	float4 edgeData : COLOR0;
};

struct PixelShaderInput {
	float4 pos : SV_POSITION;
	float3 worldPosition : POSITION0;
	nointerpolation float3 vertexWorldPositions[3] : POSITION1;
	nointerpolation float4 edgeData[3] : COLOR0;
};

[maxvertexcount(3)]
void main(triangle GeometryShaderInput input[3], inout TriangleStream<PixelShaderInput> output) {
	float3 vertexWorldPositions[3];
	float4 edgeData[3];

	for (uint i = 0; i < 3; ++i) {
		vertexWorldPositions[i] = input[i].worldPosition;
		edgeData[i] = input[i].edgeData;
	}

	for (uint i = 0; i < 3; i++) {
		PixelShaderInput element;
		element.pos = input[i].pos;
		element.worldPosition = input[i].worldPosition;
		element.vertexWorldPositions = vertexWorldPositions;
		element.edgeData = edgeData;
		output.Append(element);
	}
}
