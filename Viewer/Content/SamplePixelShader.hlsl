// Per-pixel color data passed through the pixel shader.
struct PixelShaderInput {
	float4 pos : SV_POSITION;
	float3 modelPosition : POSITION0;
	nointerpolation float3 vertexModelPositions[3] : POSITION1;
	nointerpolation float4 data0[3] : COLOR0;
	nointerpolation float3 data1[3] : COLOR3;
};

// A pass-through function for the (interpolated) color data.
float4 main(PixelShaderInput input) : SV_TARGET
{
	float minimumDistance = -1;
	uint minimumIndex = 0;
	uint indices[4] = {0, 1, 2, 0};
	if (input.data0[0].x == -1) {
		minimumDistance = distance(input.modelPosition, input.data0[0].yzw);
	}
	else {
		for (uint i = 0; i < 3; ++i) {
			if (input.data0[i].x == 1) {
				float3 unfoldedRoot = input.data0[i].yzw;
				float3 vertex0 = input.vertexModelPositions[indices[i]];
				float3 vertex1 = input.vertexModelPositions[indices[i + 1]];
				float3 edgeVector = vertex1 - vertex0;
				float3 threshold0 = vertex0 + (input.data1[i].x * edgeVector);
				float3 threshold1 = vertex0 + (input.data1[i].y * edgeVector);
				float3 boundary0 = threshold0 - unfoldedRoot;
				float3 boundary1 = threshold1 - unfoldedRoot;
				float3 probe = input.modelPosition - unfoldedRoot;
				float3 crossProduct0 = cross(boundary0, probe);
				float3 crossProduct1 = cross(boundary1, probe);
				float dotProduct = dot(crossProduct0, crossProduct1);
				if (dotProduct <= 0) {
					// The point is inside the arc.
					float currentDistance = distance(input.modelPosition, unfoldedRoot) + input.data1[i].z;
					if (minimumDistance < 0 || currentDistance < minimumDistance) {
						minimumDistance = currentDistance;
					}
				}
			}
		}
	}
	if (minimumDistance < 0) {
		return float4(0, 0, 0, 1);
	}
	else {
		float value = minimumDistance;
		return float4(value, value, value, 1);
	}
}
