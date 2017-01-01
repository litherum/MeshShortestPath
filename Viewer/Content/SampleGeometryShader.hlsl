struct GeometryShaderInput {
	float4 pos : SV_POSITION;
	float3 modelPosition : POSITION0;
	float4 data0 : COLOR0;
	float3 data1 : COLOR1;
};

struct PixelShaderInput {
	float4 pos : SV_POSITION;
	float3 modelPosition : POSITION0;
	nointerpolation float3 vertexModelPositions[3] : POSITION1;
	nointerpolation float4 data0[3] : COLOR0;
	nointerpolation float3 data1[3] : COLOR3;
};

[maxvertexcount(3)]
void main(triangle GeometryShaderInput input[3], inout TriangleStream<PixelShaderInput> output) {
	float3 vertexModelPositions[3];
	float4 data0[3];
	float3 data1[3];

	for (uint i = 0; i < 3; ++i) {
		vertexModelPositions[i] = input[i].modelPosition;
		data0[i] = input[i].data0;
		data1[i] = input[i].data1;
	}

	for (uint i = 0; i < 3; i++) {
		PixelShaderInput element;
		element.pos = input[i].pos;
		element.modelPosition = input[i].modelPosition;
		element.vertexModelPositions = vertexModelPositions;
		element.data0 = data0;
		element.data1 = data1;
		output.Append(element);
	}
}
