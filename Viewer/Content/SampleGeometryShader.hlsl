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

struct PixelShaderInput {
	float4 pos : SV_POSITION;
	float3 modelPosition : POSITION0;
	nointerpolation float3 vertexModelPositions[3] : POSITION1;
	nointerpolation float4 data0[3] : COLOR0;
	nointerpolation float4 data1[3] : COLOR4;
	nointerpolation float4 data2[3] : COLOR8;
	nointerpolation float4 data3[3] : COLOR12;
	nointerpolation float4 data4[3] : COLOR16;
	nointerpolation float4 data5[3] : COLOR20;
	nointerpolation float data6[3] : COLOR24;
};

[maxvertexcount(3)]
void main(triangle GeometryShaderInput input[3], inout TriangleStream<PixelShaderInput> output) {
	float3 vertexModelPositions[3];
	float4 data0[3];
	float4 data1[3];
	float4 data2[3];
	float4 data3[3];
	float4 data4[3];
	float4 data5[3];
	float data6[3];

	for (uint i = 0; i < 3; ++i) {
		vertexModelPositions[i] = input[i].modelPosition;
		data0[i] = input[i].data0;
		data1[i] = input[i].data1;
		data2[i] = input[i].data2;
		data3[i] = input[i].data3;
		data4[i] = input[i].data4;
		data5[i] = input[i].data5;
		data6[i] = input[i].data6;
	}

	for (uint i = 0; i < 3; i++) {
		PixelShaderInput element;
		element.pos = input[i].pos;
		element.modelPosition = input[i].modelPosition;
		element.vertexModelPositions = vertexModelPositions;
		element.data0 = data0;
		element.data1 = data1;
		element.data2 = data2;
		element.data3 = data3;
		element.data4 = data4;
		element.data5 = data5;
		element.data6 = data6;
		output.Append(element);
	}
}
