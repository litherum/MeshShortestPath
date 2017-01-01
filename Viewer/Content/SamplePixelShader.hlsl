// Per-pixel color data passed through the pixel shader.
struct PixelShaderInput {
	float4 pos : SV_POSITION;
	float3 worldPosition : POSITION0;
	nointerpolation float3 vertexWorldPositions[3] : POSITION1;
	nointerpolation float4 data0[3] : COLOR0;
	nointerpolation float3 data1[3] : COLOR3;
};

// A pass-through function for the (interpolated) color data.
float4 main(PixelShaderInput input) : SV_TARGET
{
	float minimum = -1;
	uint minimumIndex = 0;
	uint indices[4] = { 0, 1, 2, 0 };
	for (uint i = 0; i < 3; ++i) {
		// FIXME: Not quite sure what the best visualization would be here.
		float3 v0 = input.vertexWorldPositions[indices[i]];
		float3 v1 = input.vertexWorldPositions[indices[i + 1]];
		float3 a = input.worldPosition - v0;
		float3 b = v1 - v0;
		float scalar = dot(a, b) / dot(b, b);
		scalar = clamp(scalar, 0, 1);
		float3 a1 = scalar * b;
		float d = distance(a, a1);
		if (minimum < 0 || d < minimum) {
			minimum = d;
			minimumIndex = i;
		}
	}
	return float4(input.data0[minimumIndex].yzw, 1.0f);
}
