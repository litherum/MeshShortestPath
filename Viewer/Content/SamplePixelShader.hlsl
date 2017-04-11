// Per-pixel color data passed through the pixel shader.
struct PixelShaderInput {
	float4 pos : SV_POSITION;
	float3 modelPosition : POSITION0;
	nointerpolation float3 vertexModelPositions[3] : POSITION1;
	nointerpolation float4 data0[3] : COLOR0;
	nointerpolation float4 data1[3] : COLOR4;
	nointerpolation float4 data2[3] : COLOR8;
	nointerpolation float4 data3[3] : COLOR12;
	nointerpolation float3 data4[3] : COLOR16;
};

float2 computeDistance(float3 modelPosition, float3 vertex0, float3 vertex1, float3 unfoldedRoot, float beginpointFraction, float endpointFraction, float depth) {
	float3 edgeVector = vertex1 - vertex0;
	float3 threshold0 = vertex0 + (beginpointFraction * edgeVector);
	float3 threshold1 = vertex0 + (endpointFraction * edgeVector);
	float3 boundary0 = threshold0 - unfoldedRoot;
	float3 boundary1 = threshold1 - unfoldedRoot;
	float3 probe = modelPosition - unfoldedRoot;
	float3 crossProduct0 = cross(boundary0, probe);
	float3 crossProduct1 = cross(boundary1, probe);
	float dotProduct = dot(crossProduct0, crossProduct1);
	float currentDistance = distance(modelPosition, unfoldedRoot) + depth;
	return float2(dotProduct <= 0 ? 1.0 : 0.0, currentDistance);
}

// A pass-through function for the (interpolated) color data.
float4 main(PixelShaderInput input) : SV_TARGET
{
	float minimumDistance = -1;
	uint minimumIndex = 0;
	uint indices[4] = {0, 1, 2, 0};
	for (uint i = 0; i < 3; ++i) {
		float3 vertex0 = input.vertexModelPositions[indices[i]];
		float3 vertex1 = input.vertexModelPositions[indices[i + 1]];
		float intervalCount = input.data0[i].x;
		if (intervalCount == 1) {
			float2 computedDistance = computeDistance(input.modelPosition, vertex0, vertex1, input.data0[i].yzw, input.data1[i].x, input.data1[i].y, input.data1[i].z);
			if (computedDistance.x > 0.5) {
				// The point is inside the arc.
				float currentDistance = computedDistance.y;
				if (minimumDistance < 0 || currentDistance < minimumDistance)
					minimumDistance = currentDistance;
			}
		}
		else if (intervalCount == 2) {
			float2 computedDistance1 = computeDistance(input.modelPosition, vertex0, vertex1, input.data0[i].yzw, input.data1[i].x, input.data1[i].y, input.data1[i].z);
			float2 computedDistance2 = computeDistance(input.modelPosition, vertex0, vertex1, float3(input.data1[i].w, input.data2[i].xy), input.data2[i].z, input.data2[i].w, input.data3[i].x);
			if (computedDistance1.x > 0.5) {
				// The point is inside the arc.
				float currentDistance = computedDistance1.y;
				if (minimumDistance < 0 || currentDistance < minimumDistance)
					minimumDistance = currentDistance;
			}
			if (computedDistance2.x > 0.5) {
				// The point is inside the arc.
				float currentDistance = computedDistance2.y;
				if (minimumDistance < 0 || currentDistance < minimumDistance)
					minimumDistance = currentDistance;
			}
		}
		else if (intervalCount == 3) {
			float2 computedDistance1 = computeDistance(input.modelPosition, vertex0, vertex1, input.data0[i].yzw, input.data1[i].x, input.data1[i].y, input.data1[i].z);
			float2 computedDistance2 = computeDistance(input.modelPosition, vertex0, vertex1, float3(input.data1[i].w, input.data2[i].xy), input.data2[i].z, input.data2[i].w, input.data3[i].x);
			float2 computedDistance3 = computeDistance(input.modelPosition, vertex0, vertex1, input.data3[i].yzw, input.data4[i].x, input.data4[i].y, input.data4[i].z);
			if (computedDistance1.x > 0.5) {
				// The point is inside the arc.
				float currentDistance = computedDistance1.y;
				if (minimumDistance < 0 || currentDistance < minimumDistance)
					minimumDistance = currentDistance;
			}
			if (computedDistance2.x > 0.5) {
				// The point is inside the arc.
				float currentDistance = computedDistance2.y;
				if (minimumDistance < 0 || currentDistance < minimumDistance)
					minimumDistance = currentDistance;
			}
			if (computedDistance3.x > 0.5) {
				// The point is inside the arc.
				float currentDistance = computedDistance3.y;
				if (minimumDistance < 0 || currentDistance < minimumDistance)
					minimumDistance = currentDistance;
			}
		}
	}
	if (minimumDistance < 0) {
		return float4(0, 0, 0, 1);
	}
	else {
		return float4(sin(minimumDistance * 100), 0, 0, 1);
	}
}
