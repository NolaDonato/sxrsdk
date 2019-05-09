Radiance @LightType(Surface s, in U@LightType data, int index)
{
    vec4 L = u_view * vec4(data.world_direction.xyz, 0.0);
	float attenuation = 1.0;
    vec3 lightdir = normalize(L.xyz);

 #ifdef HAS_SHADOWS
    vec4 shadowCoord = @LightType_shadow_position[index];
    if ((data.shadow_map_index >= 0.0) && (shadowCoord.w > 0.0))
	{
        float nDotL = max(dot(s.viewspaceNormal, lightdir), 0.0);
        float bias = 0.001 * tan(acos(nDotL));
        bias = clamp(bias, 0.0, 0.01);
        float shadow = calculateShadow(shadowCoord, u_shadow_maps, data.shadow_map_index, bias);
        if (shadow < 1.0)
        {
            attenuation = data.shadow_level + shadow;
        }
    }
#endif
 	return Radiance(data.ambient_intensity.xyz,
					data.diffuse_intensity.xyz,
					data.specular_intensity.xyz,
					-lightdir,
					attenuation);
		
					
					
}