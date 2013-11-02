/*
 * Copyright (C) Volition, Inc. 1999.  All rights reserved.
 *
 * All source code herein is the property of Volition, Inc. You may not sell 
 * or otherwise commercially exploit the source or things you created based on the 
 * source.
 *
*/ 




#define MODEL_LIB

#include "model/model.h"
#include "math/vecmat.h"
#include "graphics/tmapper.h"
#include "math/fvi.h"
#include "model/modelsinc.h"
#include "cmdline/cmdline.h"

#define TOL		1E-4
#define DIST_TOL	1.0




void model_collide_free_point_list(/*model_collision * MC_struct_ptr*/)
{
//	mprintf(("modelcollide: model_collide_free_point_list\n"));
//	if (MC_struct_ptr->MC_point_list != NULL) {
//		vm_free(MC_struct_ptr->MC_point_list);
//		MC_struct_ptr->MC_point_list = NULL;
//	}
}

// allocate the point list
// NOTE: SHOULD ONLY EVER BE CALLED FROM model_allocate_interp_data()!!!
void model_collide_allocate_point_list(/*model_collision * MC_struct_ptr, */int n_points)
{
//	Assert( n_points > 0 );
//
//	mprintf(("modelcollide: model_collide_allocate_point_list\n"));
//	if (MC_struct_ptr->MC_point_list != NULL) {
//		vm_free(MC_struct_ptr->MC_point_list);
//		MC_struct_ptr->MC_point_list = NULL;
//	}
//
//	MC_struct_ptr->MC_point_list = (vec3d**) vm_malloc( sizeof(vec3d *) * n_points );
//
//	Verify( MC_struct_ptr->MC_point_list != NULL );
}

// Returns non-zero if vector from p0 to pdir 
// intersects the bounding box.
// hitpos could be NULL, so don't fill it if it is.
inline int mc_ray_boundingbox( model_collision * MC_struct_ptr, vec3d *min, vec3d *max, vec3d * p0, vec3d *pdir, vec3d *hitpos )
{

	vec3d tmp_hitpos;
	if ( hitpos == NULL )	{
		hitpos = &tmp_hitpos;
	}


	if ( MC_struct_ptr->MC_info->flags & MC_CHECK_SPHERELINE )	{

		// In the case of a sphere, just increase the size of the box by the radius 
		// of the sphere in all directions.

		vec3d sphere_mod_min, sphere_mod_max;

		sphere_mod_min.xyz.x = min->xyz.x - MC_struct_ptr->MC_info->radius;
		sphere_mod_max.xyz.x = max->xyz.x + MC_struct_ptr->MC_info->radius;
		sphere_mod_min.xyz.y = min->xyz.y - MC_struct_ptr->MC_info->radius;
		sphere_mod_max.xyz.y = max->xyz.y + MC_struct_ptr->MC_info->radius;
		sphere_mod_min.xyz.z = min->xyz.z - MC_struct_ptr->MC_info->radius;
		sphere_mod_max.xyz.z = max->xyz.z + MC_struct_ptr->MC_info->radius;

		return fvi_ray_boundingbox( &sphere_mod_min, &sphere_mod_max, p0, pdir, hitpos );
	} else {
		return fvi_ray_boundingbox( min, max, p0, pdir, hitpos );
	}	
}



// ----- 
// mc_check_face
// nv -- number of vertices
// verts -- actual vertices
// plane_pnt -- A point on the plane.  Could probably use the first vertex.
// plane_norm -- normal of the plane
// uvl_list -- list of uv coords for the poly.
// ntmap -- The tmap index into the model's textures array.
//
// detects whether or not a vector has collided with a polygon.  vector points stored in global
// MC_struct_ptr->MC_p0 and MC_struct_ptr->MC_p1.  Results stored in global model_collision * MC_struct_ptr.

static void mc_check_face(model_collision * MC_struct_ptr, int nv, vec3d **verts, vec3d *plane_pnt, float face_rad, vec3d *plane_norm, uv_pair *uvl_list, int ntmap, ubyte *poly, bsp_collision_leaf* bsp_leaf)
{
	vec3d	hit_point;
	float		dist;
	float		u, v;

	// Check to see if poly is facing away from ray.  If so, don't bother
	// checking it.
	if (vm_vec_dot(&MC_struct_ptr->MC_direction,plane_norm) > 0.0f)	{
		return;
	}

	// Find the intersection of this ray with the plane that the poly
	dist = fvi_ray_plane(NULL, plane_pnt, plane_norm, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, 0.0f);

	if ( dist < 0.0f ) return; // If the ray is behind the plane there is no collision
	if ( !(MC_struct_ptr->MC_info->flags & MC_CHECK_RAY) && (dist > 1.0f) ) return; // The ray isn't long enough to intersect the plane

	// If the ray hits, but a closer intersection has already been found, return
	if ( MC_struct_ptr->MC_info->num_hits && (dist >= MC_struct_ptr->MC_info->hit_dist ) ) return;

	// Find the hit point
	vm_vec_scale_add( &hit_point, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, dist );
	
	// Check to see if the point of intersection is on the plane.  If so, this
	// also finds the uv's where the ray hit.
	if ( fvi_point_face(&hit_point, nv, verts, plane_norm, &u,&v, uvl_list ) )	{
		MC_struct_ptr->MC_info->hit_dist = dist;

		MC_struct_ptr->MC_info->hit_point = hit_point;
		MC_struct_ptr->MC_info->hit_submodel = MC_struct_ptr->MC_submodel;

		MC_struct_ptr->MC_info->hit_normal = *plane_norm;

		if ( uvl_list )	{
			MC_struct_ptr->MC_info->hit_u = u;
			MC_struct_ptr->MC_info->hit_v = v;
			MC_struct_ptr->MC_info->hit_bitmap = MC_struct_ptr->MC_polymodel->maps[ntmap].textures[TM_BASE_TYPE].GetTexture();
		}
		
		if(ntmap >= 0){
			MC_struct_ptr->MC_info->t_poly = poly;
			MC_struct_ptr->MC_info->f_poly = NULL;
		} else {
			MC_struct_ptr->MC_info->t_poly = NULL;
			MC_struct_ptr->MC_info->f_poly = poly;
		}

		MC_struct_ptr->MC_info->bsp_leaf = bsp_leaf;

//		mprintf(( "Bing!\n" ));

		MC_struct_ptr->MC_info->num_hits++;
	}
}

// ----------------------------------------------------------------------------------------------------------
// check face with spheres
//
//	inputs:	nv				=>		number of vertices
//				verts			=>		array of vertices
//				plane_pnt	=>		center point in plane (about which radius is measured)
//				face_rad		=>		radius of face 
//				plane_norm	=>		normal of face
static void mc_check_sphereline_face( model_collision * MC_struct_ptr, int nv, vec3d ** verts, vec3d * plane_pnt, float face_rad, vec3d * plane_norm, uv_pair * uvl_list, int ntmap, ubyte *poly, bsp_collision_leaf *bsp_leaf)
{
	vec3d	hit_point;
	float		u, v;
	float		delta_t;			// time sphere takes to cross from one side of plane to the other
	float		face_t;			// time at which face touches plane
									// NOTE all times are normalized so that t = 1.0 at the end of the frame
	int		check_face = 1;		// assume we'll check the face.
	int		check_edges = 1;		// assume we'll check the edges.
	
	// Check to see if poly is facing away from ray.  If so, don't bother
	// checking it.

	if (vm_vec_dot(&MC_struct_ptr->MC_direction,plane_norm) > 0.0f)	{
		return;
	}

	// Find the intersection of this sphere with the plane of the poly
	if ( !fvi_sphere_plane( &hit_point, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, MC_struct_ptr->MC_info->radius, plane_norm, plane_pnt, &face_t, &delta_t ) ) {
		return;
	}

	if ( face_t < 0 || face_t > 1) {
		check_face = 0;		// If the ray is behind the plane there is no collision
	}

	if ( !(MC_struct_ptr->MC_info->flags & MC_CHECK_RAY) && (face_t > 1.0f) ) {
		check_face = 0;		// The ray isn't long enough to intersect the plane
	}

	// If the ray hits, but a closer intersection has already been found, don't check face
	if ( MC_struct_ptr->MC_info->num_hits && (face_t >= MC_struct_ptr->MC_info->hit_dist ) ) {
		check_face = 0;		// The ray isn't long enough to intersect the plane
	}


	vec3d temp_sphere;
	vec3d temp_dir;
	float temp_dist;
	// DA 11/5/97  Above is used to test distance between hit_point and sphere_hit_point.
	// This can be as large as 0.003 on a unit sphere.  I suspect that with larger spheres,
	// both the relative and absolute error decrease, but this should still be checked for the
	// case of larger spheres (about 5-10 units).  The error also depends on the geometry of the 
	// object we're colliding against, but I think to a lesser degree.

	if ( check_face )	{
		// Find the time of the sphere surface touches the plane
		// If this is within the collision window, check to see if we hit a face
		if ( fvi_point_face(&hit_point, nv, verts, plane_norm, &u, &v, uvl_list) ) {

			MC_struct_ptr->MC_info->hit_dist = face_t;
			MC_struct_ptr->MC_info->hit_point = hit_point;
			MC_struct_ptr->MC_info->hit_normal = *plane_norm;
			MC_struct_ptr->MC_info->hit_submodel = MC_struct_ptr->MC_submodel;
			MC_struct_ptr->MC_info->edge_hit = 0;

			if ( uvl_list )	{
				MC_struct_ptr->MC_info->hit_u = u;
				MC_struct_ptr->MC_info->hit_v = v;
				MC_struct_ptr->MC_info->hit_bitmap = MC_struct_ptr->MC_polymodel->maps[ntmap].textures[TM_BASE_TYPE].GetTexture();
			}

			if(ntmap >= 0){
				MC_struct_ptr->MC_info->t_poly = poly;
				MC_struct_ptr->MC_info->f_poly = NULL;
			} else {
				MC_struct_ptr->MC_info->t_poly = NULL;
				MC_struct_ptr->MC_info->f_poly = poly;
			}

			MC_struct_ptr->MC_info->bsp_leaf = bsp_leaf;

			MC_struct_ptr->MC_info->num_hits++;
			check_edges = 0;
			vm_vec_scale_add( &temp_sphere, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, MC_struct_ptr->MC_info->hit_dist );
			temp_dist = vm_vec_dist( &temp_sphere, &hit_point );
			if ( (temp_dist - DIST_TOL > MC_struct_ptr->MC_info->radius) || (temp_dist + DIST_TOL < MC_struct_ptr->MC_info->radius) ) {
				// get Andsager
				//mprintf(("Estimated radius error: Estimate %f, actual %f MC_struct_ptr->MC_info->radius\n", temp_dist, MC_struct_ptr->MC_info->radius));
			}
			vm_vec_sub( &temp_dir, &hit_point, &temp_sphere );
			// Assert( vm_vec_dotprod( &temp_dir, &MC_struct_ptr->MC_direction ) > 0 );
		}
	}


	if ( check_edges ) {

		// Either (face_t) is out of range or we miss the face
		// Check for sphere hitting edge

		// If checking shields, we *still* need to check edges

		// First check whether sphere can hit edge in allowed time range
		if ( face_t > 1.0f || face_t+delta_t < 0.0f )	{
			return;
		}

		// this is where we need another test to cull checking for edges
		// PUT TEST HERE

		// check each edge to see if we hit, find the closest edge
		// MC_struct_ptr->MC_info->hit_dist stores the best edge time of *all* faces
		float sphere_time;
		if ( fvi_polyedge_sphereline(&hit_point, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, MC_struct_ptr->MC_info->radius, nv, verts, &sphere_time)) {

			Assert( sphere_time >= 0.0f );
			vm_vec_scale_add( &temp_sphere, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, sphere_time );
			temp_dist = vm_vec_dist( &temp_sphere, &hit_point );
			if ( (temp_dist - DIST_TOL > MC_struct_ptr->MC_info->radius) || (temp_dist + DIST_TOL < MC_struct_ptr->MC_info->radius) ) {
				// get Andsager
				//mprintf(("Estimated radius error: Estimate %f, actual %f MC_struct_ptr->MC_info->radius\n", temp_dist, MC_struct_ptr->MC_info->radius));
			}
			vm_vec_sub( &temp_dir, &hit_point, &temp_sphere );
//			Assert( vm_vec_dotprod( &temp_dir, &MC_struct_ptr->MC_direction ) > 0 );

			if ( (MC_struct_ptr->MC_info->num_hits==0) || (sphere_time < MC_struct_ptr->MC_info->hit_dist) ) {
				// This is closer than best so far
				MC_struct_ptr->MC_info->hit_dist = sphere_time;
				MC_struct_ptr->MC_info->hit_point = hit_point;
				MC_struct_ptr->MC_info->hit_submodel = MC_struct_ptr->MC_submodel;
				MC_struct_ptr->MC_info->edge_hit = 1;
				MC_struct_ptr->MC_info->hit_bitmap = MC_struct_ptr->MC_polymodel->maps[ntmap].textures[TM_BASE_TYPE].GetTexture();

				if(ntmap >= 0){
					MC_struct_ptr->MC_info->t_poly = poly;
					MC_struct_ptr->MC_info->f_poly = NULL;
				} else {
					MC_struct_ptr->MC_info->t_poly = NULL;
					MC_struct_ptr->MC_info->f_poly = poly;
				}

				MC_struct_ptr->MC_info->num_hits++;

			//	nprintf(("Physics", "edge sphere time: %f, normal: (%f, %f, %f) hit_point: (%f, %f, %f)\n", sphere_time,
			//		MC_struct_ptr->MC_info->hit_normal.xyz.x, MC_struct_ptr->MC_info->hit_normal.xyz.y, MC_struct_ptr->MC_info->hit_normal.xyz.z,
			//		hit_point.xyz.x, hit_point.xyz.y, hit_point.xyz.z));
			} else  {	// Not best so far
				Assert(MC_struct_ptr->MC_info->num_hits>0);
				MC_struct_ptr->MC_info->num_hits++;
			}
		}
	}
}


// Point list
// +0      int         id
// +4      int         size
// +8      int         n_verts
// +12     int         n_norms
// +16     int         offset from start of chunk to vertex data
// +20     n_verts*char    norm_counts
// +offset             vertex data. Each vertex n is a point followed by norm_counts[n] normals.     
void model_collide_defpoints(model_collision * MC_struct_ptr, ubyte * p)
{
	int n;
	int nverts = w(p+8);	
	int offset = w(p+16);	

	ubyte * normcount = p+20;
	vec3d *src = vp(p+offset);
	
//	Assert( MC_struct_ptr->MC_point_list != NULL );
	MC_struct_ptr->MC_point_list.clear();

	for (n=0; n<nverts; n++ ) {
//		MC_struct_ptr->MC_point_list[n] = src;
		MC_struct_ptr->MC_point_list.push_back(src);

		src += normcount[n]+1;
	} 
}

int model_collide_parse_bsp_defpoints(model_collision * MC_struct_ptr, ubyte * p)
{
	int n;
	int nverts = w(p+8);	
	int offset = w(p+16);

	ubyte * normcount = p+20;
	vec3d *src = vp(p+offset);

//	Assert( MC_struct_ptr->MC_point_list != NULL );
	MC_struct_ptr->MC_point_list.clear();

	for (n=0; n<nverts; n++ ) {
//		MC_struct_ptr->MC_point_list[n] = src;
		MC_struct_ptr->MC_point_list.push_back(src);

		src += normcount[n]+1;
	}

	return nverts;
}

// Flat Poly
// +0      int         id
// +4      int         size 
// +8      vec3d      normal
// +20     vec3d      center
// +32     float       radius
// +36     int         nverts
// +40     byte        red
// +41     byte        green
// +42     byte        blue
// +43     byte        pad
// +44     nverts*int  vertlist
void model_collide_flatpoly(model_collision * MC_struct_ptr, ubyte * p)
{
	int i;
	int nv;
	vec3d *points[TMAP_MAX_VERTS];
	short *verts;

	nv = w(p+36);
	if ( nv < 0 ) return;

	if ( nv > TMAP_MAX_VERTS ) {
		Int3();
		return;
	}

	verts = (short *)(p+44);

	for (i=0;i<nv;i++)	{
		points[i] = MC_struct_ptr->MC_point_list[verts[i*2]];
	}

	if ( MC_struct_ptr->MC_info->flags & MC_CHECK_SPHERELINE )	{
		mc_check_sphereline_face(MC_struct_ptr, nv, points, vp(p+20), fl(p+32), vp(p+8), NULL, -1, p, NULL);
	} else {
		mc_check_face(MC_struct_ptr, nv, points, vp(p+20), fl(p+32), vp(p+8), NULL, -1, p, NULL);
	}
}


// Textured Poly
// +0      int         id
// +4      int         size 
// +8      vec3d      normal
// +20     vec3d      normal_point
// +32     int         tmp = 0
// +36     int         nverts
// +40     int         tmap_num
// +44     nverts*(model_tmap_vert) vertlist (n,u,v)
void model_collide_tmappoly(model_collision * MC_struct_ptr, ubyte * p)
{
	int i;
	int nv;
	uv_pair uvlist[TMAP_MAX_VERTS];
	vec3d *points[TMAP_MAX_VERTS];
	model_tmap_vert *verts;

	nv = w(p+36);
	if ( nv < 0 ) return;

	if ( nv > TMAP_MAX_VERTS ) {
		Int3();
		return;
	}

	int tmap_num = w(p+40);
	Assert(tmap_num >= 0 && tmap_num < MAX_MODEL_TEXTURES);	// Goober5000

	if ( (!(MC_struct_ptr->MC_info->flags & MC_CHECK_INVISIBLE_FACES)) && (MC_struct_ptr->MC_polymodel->maps[tmap_num].textures[TM_BASE_TYPE].GetTexture() < 0) )	{
		// Don't check invisible polygons.
		//SUSHI: Unless $collide_invisible is set.
		if (!(MC_struct_ptr->MC_polymodel->submodel[MC_struct_ptr->MC_submodel].collide_invisible))
			return;
	}

	verts = (model_tmap_vert *)(p+44);

	for (i=0;i<nv;i++)	{
		points[i] = MC_struct_ptr->MC_point_list[verts[i].vertnum];
		uvlist[i].u = verts[i].u;
		uvlist[i].v = verts[i].v;
	}

	if ( MC_struct_ptr->MC_info->flags & MC_CHECK_SPHERELINE )	{
		mc_check_sphereline_face(MC_struct_ptr, nv, points, vp(p+20), fl(p+32), vp(p+8), uvlist, tmap_num, p, NULL);
	} else {
		mc_check_face(MC_struct_ptr, nv, points, vp(p+20), fl(p+32), vp(p+8), uvlist, tmap_num, p, NULL);
	}
}


// Sortnorms
// +0      int         id
// +4      int         size 
// +8      vec3d      normal
// +20     vec3d      center
// +32     float       radius
// 36     int     front offset
// 40     int     back offset
// 44     int     prelist offset
// 48     int     postlist offset
// 52     int     online offset

int model_collide_sub( model_collision * MC_struct_ptr, void *model_ptr );

inline void model_collide_sortnorm(model_collision * MC_struct_ptr, ubyte * p)
{
	int frontlist = w(p+36);
	int backlist = w(p+40);
	int prelist = w(p+44);
	int postlist = w(p+48);
	int onlist = w(p+52);
	vec3d hitpos;

	if ( MC_struct_ptr->MC_polymodel->version >= 2000 )	{
		if ( mc_ray_boundingbox(MC_struct_ptr, vp(p+56), vp(p+68), &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, &hitpos) )	{
			if ( !(MC_struct_ptr->MC_info->flags & MC_CHECK_RAY) && (vm_vec_dist(&hitpos, &MC_struct_ptr->MC_p0) > MC_struct_ptr->MC_mag) ) {
				return;
			}
		} else {
			return;
		}
	}

	if (prelist) model_collide_sub(MC_struct_ptr, p+prelist);
	if (backlist) model_collide_sub(MC_struct_ptr, p+backlist);
	if (onlist) model_collide_sub(MC_struct_ptr, p+onlist);
	if (frontlist) model_collide_sub(MC_struct_ptr, p+frontlist);
	if (postlist) model_collide_sub(MC_struct_ptr, p+postlist);
}

//calls the object interpreter to render an object.  The object renderer
//is really a seperate pipeline. returns true if drew
inline int model_collide_sub(model_collision * MC_struct_ptr, void *model_ptr )
{
	ubyte *p = (ubyte *)model_ptr;
	int chunk_type, chunk_size;
	vec3d hitpos;

	chunk_type = w(p);
	chunk_size = w(p+4);

	while (chunk_type != OP_EOF)	{

//		mprintf(( "Processing chunk type %d, len=%d\n", chunk_type, chunk_size ));

		switch (chunk_type) {
		case OP_DEFPOINTS:	model_collide_defpoints(MC_struct_ptr, p); break;
		case OP_FLATPOLY:		model_collide_flatpoly(MC_struct_ptr, p); break;
		case OP_TMAPPOLY:		model_collide_tmappoly(MC_struct_ptr, p); break;
		case OP_SORTNORM:		model_collide_sortnorm(MC_struct_ptr, p); break;
		case OP_BOUNDBOX:	
			if ( mc_ray_boundingbox( MC_struct_ptr, vp(p+8), vp(p+20), &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, &hitpos ) )	{
				if ( !(MC_struct_ptr->MC_info->flags & MC_CHECK_RAY) && (vm_vec_dist(&hitpos, &MC_struct_ptr->MC_p0) > MC_struct_ptr->MC_mag) ) {
					// The ray isn't long enough to intersect the bounding box
					return 1;
				}
			} else {
				return 1;
			}
			break;
		default:
			mprintf(( "Bad chunk type %d, len=%d in model_collide_sub\n", chunk_type, chunk_size ));
			Int3();		// Bad chunk type!
			return 0;
		}
		p += chunk_size;
		chunk_type = w(p);
		chunk_size = w(p+4);
	}
	return 1;
}

void model_collide_bsp_poly(model_collision * MC_struct_ptr, bsp_collision_tree *tree, int leaf_index)
{
	int i;
	int tested_leaf = leaf_index;
	uv_pair uvlist[TMAP_MAX_VERTS];
	vec3d *points[TMAP_MAX_VERTS];

	while ( tested_leaf >= 0 ) {
		bsp_collision_leaf *leaf = &tree->leaf_list[tested_leaf];

		bool flat_poly = false;
		int vert_start = leaf->vert_start;
		int nv = leaf->num_verts;

		if ( leaf->tmap_num < MAX_MODEL_TEXTURES ) {
			if ( (!(MC_struct_ptr->MC_info->flags & MC_CHECK_INVISIBLE_FACES)) && (MC_struct_ptr->MC_polymodel->maps[leaf->tmap_num].textures[TM_BASE_TYPE].GetTexture() < 0) )	{
				// Don't check invisible polygons.
				//SUSHI: Unless $collide_invisible is set.
				if (!(MC_struct_ptr->MC_polymodel->submodel[MC_struct_ptr->MC_submodel].collide_invisible))
					return;
			}
		} else {
			flat_poly = true;
		}

		int vert_num;
		for ( i = 0; i < nv; ++i ) {
			vert_num = tree->vert_list[vert_start+i].vertnum;
			points[i] = &tree->point_list[vert_num];

			uvlist[i].u = tree->vert_list[vert_start+i].u;
			uvlist[i].v = tree->vert_list[vert_start+i].v;
		}

		if ( flat_poly ) {
			if ( MC_struct_ptr->MC_info->flags & MC_CHECK_SPHERELINE ) {
				mc_check_sphereline_face(MC_struct_ptr, nv, points, &leaf->plane_pnt, leaf->face_rad, &leaf->plane_norm, NULL, -1, NULL, leaf);
			} else {
				mc_check_face(MC_struct_ptr, nv, points, &leaf->plane_pnt, leaf->face_rad, &leaf->plane_norm, NULL, -1, NULL, leaf);
			}
		} else {
			if ( MC_struct_ptr->MC_info->flags & MC_CHECK_SPHERELINE ) {
				mc_check_sphereline_face(MC_struct_ptr, nv, points, &leaf->plane_pnt, leaf->face_rad, &leaf->plane_norm, uvlist, leaf->tmap_num, NULL, leaf);
			} else {
				mc_check_face(MC_struct_ptr, nv, points, &leaf->plane_pnt, leaf->face_rad, &leaf->plane_norm, uvlist, leaf->tmap_num, NULL, leaf);
			}
		}

		tested_leaf = leaf->next;
	}
}

inline void model_collide_bsp(model_collision * MC_struct_ptr, bsp_collision_tree *tree, int node_index)
{
	if ( tree->node_list == NULL || tree->n_verts <= 0) {
		return;
	}

	bsp_collision_node *node = &tree->node_list[node_index];
	vec3d hitpos;

	// check the bounding box of this node. if it passes, check left and right children
	if ( mc_ray_boundingbox( MC_struct_ptr, &node->min, &node->max, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, &hitpos ) ) {
		if ( !(MC_struct_ptr->MC_info->flags & MC_CHECK_RAY) && (vm_vec_dist(&hitpos, &MC_struct_ptr->MC_p0) > MC_struct_ptr->MC_mag) ) {
			// The ray isn't long enough to intersect the bounding box
			return;
		}

		if ( node->leaf >= 0 ) {
			model_collide_bsp_poly(MC_struct_ptr, tree, node->leaf);
		} else {
			if ( node->back >= 0 ) model_collide_bsp(MC_struct_ptr, tree, node->back);
			if ( node->front >= 0 ) model_collide_bsp(MC_struct_ptr, tree, node->front);
		}
	}
}

void model_collide_parse_bsp_tmappoly(bsp_collision_leaf *leaf, SCP_vector<model_tmap_vert> *vert_buffer, void *model_ptr)
{
	ubyte *p = (ubyte *)model_ptr;

	int i;
	int nv;
	model_tmap_vert *verts;

	nv = w(p+36);

	if ( nv < 0 ) return;

	if ( nv > TMAP_MAX_VERTS ) {
		Int3();
		return;
	}

	int tmap_num = w(p+40);

	Assert(tmap_num >= 0 && tmap_num < MAX_MODEL_TEXTURES);

	verts = (model_tmap_vert *)(p+44);

	leaf->tmap_num = (ubyte)tmap_num;
	leaf->num_verts = (ubyte)nv;
	leaf->vert_start = vert_buffer->size();

	vec3d *plane_pnt = vp(p+20);
	float face_rad = fl(p+32);
	vec3d *plane_norm = vp(p+8);

	leaf->plane_pnt = *plane_pnt;
	leaf->face_rad = face_rad;
	leaf->plane_norm = *plane_norm;

	for ( i = 0; i < nv; ++i ) {
		vert_buffer->push_back(verts[i]);
	}
}

void model_collide_parse_bsp_flatpoly(bsp_collision_leaf *leaf, SCP_vector<model_tmap_vert> *vert_buffer, void *model_ptr)
{
	ubyte *p = (ubyte *)model_ptr;

	int i;
	int nv;
	short *verts;

	nv = w(p+36);

	if ( nv < 0 ) return;

	if ( nv > TMAP_MAX_VERTS ) {
		Int3();
		return;
	}

	verts = (short *)(p+44);

	leaf->tmap_num = 255;
	leaf->num_verts = (ubyte)nv;
	leaf->vert_start = vert_buffer->size();

	vec3d *plane_pnt = vp(p+20);
	float face_rad = fl(p+32);
	vec3d *plane_norm = vp(p+8);

	leaf->plane_pnt = *plane_pnt;
	leaf->face_rad = face_rad;
	leaf->plane_norm = *plane_norm;

	model_tmap_vert vert;

	for ( i = 0; i < nv; ++i ) {
		vert.vertnum = verts[i*2];
		vert.normnum = 0;
		vert.u = 0.0f;
		vert.v = 0.0f;

		vert_buffer->push_back(vert);
	}
}

void model_collide_parse_bsp(bsp_collision_tree *tree, void *model_ptr, int version)
{
	ubyte *p = (ubyte *)model_ptr;
	ubyte *next_p;

	int chunk_type = w(p);
	int chunk_size = w(p+4);

	int next_chunk_type;
	int next_chunk_size;

	model_collision MC_struct;

	Assert(chunk_type == OP_DEFPOINTS);

	int n_verts = model_collide_parse_bsp_defpoints(&MC_struct, p);

	if ( n_verts <= 0) {
		tree->point_list = NULL;
		tree->n_verts = 0;

		tree->n_nodes = 0;
		tree->node_list = NULL;

		tree->n_leaves = 0;
		tree->leaf_list = NULL;

		// finally copy the vert list.
		tree->vert_list = NULL;

		return;
	}

	p += chunk_size;

	bsp_collision_node new_node;
	bsp_collision_leaf new_leaf;

	SCP_vector<bsp_collision_node> node_buffer;
	SCP_vector<bsp_collision_leaf> leaf_buffer;
	SCP_vector<model_tmap_vert> vert_buffer;

	SCP_map<size_t, ubyte*> bsp_datap;

	node_buffer.push_back(new_node);

	size_t i = 0;
	vec3d *min;
	vec3d *max;

	bsp_datap[i] = p;

	while ( i < node_buffer.size() ) {
		p = bsp_datap[i];

		chunk_type = w(p);
		chunk_size = w(p+4);

		switch ( chunk_type ) {
		case OP_SORTNORM:
			if ( version >= 2000 ) {
				min = vp(p+56);
				max = vp(p+68);

				node_buffer[i].min = *min;
				node_buffer[i].max = *max;
			}

			node_buffer[i].leaf = -1;
			node_buffer[i].front = -1;
			node_buffer[i].back = -1;

			if ( w(p+36) ) {
				next_chunk_type = w(p+w(p+36));

				if ( next_chunk_type != OP_EOF ) {
					node_buffer.push_back(new_node);
					node_buffer[i].front = (node_buffer.size() - 1);
					bsp_datap[node_buffer[i].front] = p+w(p+36);
				}
			}

			if ( w(p+40) ) {
				next_chunk_type = w(p+w(p+40));
				
				if ( next_chunk_type != OP_EOF ) {
					node_buffer.push_back(new_node);
					node_buffer[i].back = (node_buffer.size() - 1);
					bsp_datap[node_buffer[i].back] = p+w(p+40);
				}
			}

			next_p = p + chunk_size;
			next_chunk_type = w(next_p);

			Assert( next_chunk_type == OP_EOF );

			++i;
			break;
		case OP_BOUNDBOX:
			min = vp(p+8);
			max = vp(p+20);

			node_buffer[i].min = *min;
			node_buffer[i].max = *max;

			node_buffer[i].front = -1;
			node_buffer[i].back = -1;
			node_buffer[i].leaf = -1;

			next_p = p + chunk_size;
			next_chunk_type = w(next_p);
			next_chunk_size = w(next_p+4);

			if ( next_chunk_type != OP_EOF && (next_chunk_type == OP_TMAPPOLY || next_chunk_type == OP_FLATPOLY ) ) {
				new_leaf.next = -1;

				node_buffer[i].leaf = leaf_buffer.size();	// get index of where our poly list starts in the leaf buffer

				while ( next_chunk_type != OP_EOF ) {
					if ( next_chunk_type == OP_TMAPPOLY ) {

						model_collide_parse_bsp_tmappoly(&new_leaf, &vert_buffer, next_p);

						leaf_buffer.push_back(new_leaf);

						leaf_buffer.back().next = leaf_buffer.size();
					} else if ( next_chunk_type == OP_FLATPOLY ) {
						model_collide_parse_bsp_flatpoly(&new_leaf, &vert_buffer, next_p);

						leaf_buffer.push_back(new_leaf);

						leaf_buffer.back().next = leaf_buffer.size();
					} else {
						Int3();
					}

					next_p += next_chunk_size;
					next_chunk_type = w(next_p);
					next_chunk_size = w(next_p+4);
				}

				leaf_buffer.back().next = -1;
			}

			Assert(next_chunk_type == OP_EOF);

			++i;
			break;
		}
	}

	// copy point list
	Assert(n_verts != -1);

	tree->point_list = (vec3d*)vm_malloc(sizeof(vec3d) * n_verts);

	for ( i = 0; i < (size_t)n_verts; ++i ) {
		tree->point_list[i] = *MC_struct.MC_point_list[i];
	}

	tree->n_verts = n_verts;

	// copy node info. this might be a good time to organize the nodes into a cache efficient tree layout.
	tree->n_nodes = node_buffer.size();
	tree->node_list = (bsp_collision_node*)vm_malloc(sizeof(bsp_collision_node) * node_buffer.size());
	memcpy(tree->node_list, &node_buffer[0], sizeof(bsp_collision_node) * node_buffer.size());
	node_buffer.clear();

	// copy leaves.
	tree->n_leaves = leaf_buffer.size();
	tree->leaf_list = (bsp_collision_leaf*)vm_malloc(sizeof(bsp_collision_leaf) * leaf_buffer.size());
	memcpy(tree->leaf_list, &leaf_buffer[0], sizeof(bsp_collision_leaf) * leaf_buffer.size());
	leaf_buffer.clear();

	// finally copy the vert list.
	tree->vert_list = (model_tmap_vert*)vm_malloc(sizeof(model_tmap_vert) * vert_buffer.size());
	memcpy(tree->vert_list, &vert_buffer[0], sizeof(model_tmap_vert) * vert_buffer.size());
	vert_buffer.clear();
}

bool mc_shield_check_common(model_collision * MC_struct_ptr, shield_tri	*tri)
{
	vec3d * points[3];
	vec3d hitpoint;
	 
	float dist;
	float sphere_check_closest_shield_dist = FLT_MAX;

	// Check to see if Mc_pmly is facing away from ray.  If so, don't bother
	// checking it.
	if (vm_vec_dot(&MC_struct_ptr->MC_direction,&tri->norm) > 0.0f)	{
		return false;
	}
	// get the vertices in the form the next function wants them
	for (int j = 0; j < 3; j++ )
		points[j] = &MC_struct_ptr->MC_polymodel->shield.verts[tri->verts[j]].pos;

	if (!(MC_struct_ptr->MC_info->flags & MC_CHECK_SPHERELINE) ) {	// Don't do this test for sphere colliding against shields
		// Find the intersection of this ray with the plane that the Mc_pmly
		// lies in
		dist = fvi_ray_plane(NULL, points[0],&tri->norm,&MC_struct_ptr->MC_p0,&MC_struct_ptr->MC_direction,0.0f);

		if ( dist < 0.0f ) return false; // If the ray is behind the plane there is no collision
		if ( !(MC_struct_ptr->MC_info->flags & MC_CHECK_RAY) && (dist > 1.0f) ) return false; // The ray isn't long enough to intersect the plane

		// Find the hit Mc_pmint
		vm_vec_scale_add( &hitpoint, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, dist );
	
		// Check to see if the Mc_pmint of intersection is on the plane.  If so, this
		// also finds the uv's where the ray hit.
		if ( fvi_point_face(&hitpoint, 3, points, &tri->norm, NULL,NULL,NULL ) )	{
			MC_struct_ptr->MC_info->hit_dist = dist;
			MC_struct_ptr->MC_info->shield_hit_tri = tri - MC_struct_ptr->MC_polymodel->shield.tris;
			MC_struct_ptr->MC_info->hit_point = hitpoint;
			MC_struct_ptr->MC_info->hit_normal = tri->norm;
			MC_struct_ptr->MC_info->hit_submodel = -1;
			MC_struct_ptr->MC_info->num_hits++;
			return true;		// We hit, so we're done
		}
	} else {		// Sphere check against shield
					// This needs to look at *all* shield tris and not just return after the first hit

		// HACK HACK!! The 10000.0 is the face radius, I didn't know this,
		// so I'm assume 10000 would be as big as ever.
		mc_check_sphereline_face(MC_struct_ptr, 3, points, points[0], 10000.0f, &tri->norm, NULL, 0, NULL, NULL);
		if (MC_struct_ptr->MC_info->num_hits && MC_struct_ptr->MC_info->hit_dist < sphere_check_closest_shield_dist) {

			// same behavior whether face or edge
			// normal, edge_hit, hit_point all updated thru sphereline_face
			sphere_check_closest_shield_dist = MC_struct_ptr->MC_info->hit_dist;
			MC_struct_ptr->MC_info->shield_hit_tri = tri - MC_struct_ptr->MC_polymodel->shield.tris;
			MC_struct_ptr->MC_info->hit_submodel = -1;
			MC_struct_ptr->MC_info->num_hits++;
			return true;		// We hit, so we're done
		}
	} // MC_struct_ptr->MC_info->flags & MC_CHECK_SPHERELINE else

	return false;
}

inline bool mc_check_sldc(model_collision * MC_struct_ptr, int offset)
{
	if (offset > MC_struct_ptr->MC_polymodel->sldc_size-5) //no way is this big enough
		return false;
	char *type_p = (char *)(MC_struct_ptr->MC_polymodel->shield_collision_tree+offset);
	
	// not used
	//int *size_p = (int *)(MC_struct_ptr->MC_polymodel->shield_collision_tree+offset+1);
	// split and polygons
	vec3d *minbox_p = (vec3d*)(MC_struct_ptr->MC_polymodel->shield_collision_tree+offset+5);
	vec3d *maxbox_p = (vec3d*)(MC_struct_ptr->MC_polymodel->shield_collision_tree+offset+17);

	// split
	unsigned int *front_offset_p = (unsigned int*)(MC_struct_ptr->MC_polymodel->shield_collision_tree+offset+29);
	unsigned int *back_offset_p = (unsigned int*)(MC_struct_ptr->MC_polymodel->shield_collision_tree+offset+33);

	// polygons
	unsigned int *num_polygons_p = (unsigned int*)(MC_struct_ptr->MC_polymodel->shield_collision_tree+offset+29);

	unsigned int *shld_polys = (unsigned int*)(MC_struct_ptr->MC_polymodel->shield_collision_tree+offset+33);



	// see if it fits inside our bbox
	if (!mc_ray_boundingbox( MC_struct_ptr, minbox_p, maxbox_p, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, NULL ))	{
		return false;
	}

	if (*type_p == 0) // SPLIT
	{
			return mc_check_sldc(MC_struct_ptr, offset+*front_offset_p) || mc_check_sldc(MC_struct_ptr, offset+*back_offset_p);
	}
	else
	{
		// poly list
		shield_tri	* tri;
		for (unsigned int i = 0; i < *num_polygons_p; i++)
		{
			tri = &MC_struct_ptr->MC_polymodel->shield.tris[shld_polys[i]];
						
			mc_shield_check_common(MC_struct_ptr, tri);

		} // for (unsigned int i = 0; i < leaf->num_polygons; i++)
	}

	// shouldn't be reached
	return false;
}

// checks a vector collision against a ships shield (if it has shield points defined).
inline void mc_check_shield(model_collision * MC_struct_ptr)
{
	int i;


	if ( MC_struct_ptr->MC_polymodel->shield.ntris < 1 )
		return;
	if (MC_struct_ptr->MC_polymodel->shield_collision_tree)
	{
		mc_check_sldc(MC_struct_ptr, 0); // see if we hit the SLDC
	}
	else
	{
		int o;
		for (o=0; o<8; o++ )	{
			model_octant * poct1 = &MC_struct_ptr->MC_polymodel->octants[o];

			if (!mc_ray_boundingbox( MC_struct_ptr, &poct1->min, &poct1->max, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, NULL ))	{
				continue;
			}
			
			for (i = 0; i < poct1->nshield_tris; i++) {
				shield_tri	* tri = poct1->shield_tris[i];
				mc_shield_check_common(MC_struct_ptr, tri);
			}
		}
	}//model has shield_collsion_tree
}


// This function recursively checks a submodel and its children
// for a collision with a vector.
inline void mc_check_subobj( model_collision * MC_struct_ptr, int mn )
{
	vec3d tempv;
	vec3d hitpt;		// used in bounding box check
	bsp_info * sm;
	int i;

	Assert( mn >= 0 );
	Assert( mn < MC_struct_ptr->MC_polymodel->n_models );

	if ( (mn < 0) || (mn>=MC_struct_ptr->MC_polymodel->n_models) ) return;
	
	sm = &MC_struct_ptr->MC_polymodel->submodel[mn];
	if (sm->no_collisions) return; // don't do collisions
	if (sm->nocollide_this_only) goto NoHit; // Don't collide for this model, but keep checking others

	// Rotate the world check points into the current subobject's 
	// frame of reference.
	// After this block, MC_struct_ptr->MC_p0, MC_struct_ptr->MC_p1, MC_struct_ptr->MC_direction, and MC_struct_ptr->MC_mag are correct
	// and relative to this subobjects' frame of reference.
	vm_vec_sub(&tempv, MC_struct_ptr->MC_info->p0, &MC_struct_ptr->MC_base);
	vm_vec_rotate(&MC_struct_ptr->MC_p0, &tempv, &MC_struct_ptr->MC_orient);

	vm_vec_sub(&tempv, MC_struct_ptr->MC_info->p1, &MC_struct_ptr->MC_base);
	vm_vec_rotate(&MC_struct_ptr->MC_p1, &tempv, &MC_struct_ptr->MC_orient);
	vm_vec_sub(&MC_struct_ptr->MC_direction, &MC_struct_ptr->MC_p1, &MC_struct_ptr->MC_p0);

	// If we are checking the root submodel, then we might want
	// to check the shield at this point
	if (MC_struct_ptr->MC_polymodel->detail[0] == mn)	{

		// Do a quick out on the entire bounding box of the object
		if (!mc_ray_boundingbox( MC_struct_ptr, &MC_struct_ptr->MC_polymodel->mins, &MC_struct_ptr->MC_polymodel->maxs, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, NULL))	{
			return;
		}
			
		// Check shield if we're supposed to
		if ((MC_struct_ptr->MC_info->flags & MC_CHECK_SHIELD) && (MC_struct_ptr->MC_polymodel->shield.ntris > 0 )) {
			mc_check_shield(MC_struct_ptr);
			return;
		}

	}

	if(!(MC_struct_ptr->MC_info->flags & MC_CHECK_MODEL)) return;
	
	MC_struct_ptr->MC_submodel = mn;

	// Check if the ray intersects this subobject's bounding box 	
	if (mc_ray_boundingbox(MC_struct_ptr, &sm->min, &sm->max, &MC_struct_ptr->MC_p0, &MC_struct_ptr->MC_direction, &hitpt))	{

		// The ray interects this bounding box, so we have to check all the
		// polygons in this submodel.
		if ( MC_struct_ptr->MC_info->flags & MC_ONLY_BOUND_BOX )	{
			float dist = vm_vec_dist( &MC_struct_ptr->MC_p0, &hitpt );

			if ( dist < 0.0f ) goto NoHit; // If the ray is behind the plane there is no collision
			if ( !(MC_struct_ptr->MC_info->flags & MC_CHECK_RAY) && (dist > MC_struct_ptr->MC_mag) ) goto NoHit; // The ray isn't long enough to intersect the plane

			// If the ray hits, but a closer intersection has already been found, return
			if ( MC_struct_ptr->MC_info->num_hits && (dist >= MC_struct_ptr->MC_info->hit_dist ) ) goto NoHit;

			MC_struct_ptr->MC_info->hit_dist = dist;
			MC_struct_ptr->MC_info->hit_point = hitpt;
			MC_struct_ptr->MC_info->hit_submodel = MC_struct_ptr->MC_submodel;
			MC_struct_ptr->MC_info->hit_bitmap = -1;
			MC_struct_ptr->MC_info->num_hits++;
		} else {
			if ( Cmdline_old_collision_sys ) {
				model_collide_sub(MC_struct_ptr, sm->bsp_data);
			} else {
				model_collide_bsp(MC_struct_ptr, model_get_bsp_collision_tree(sm->collision_tree_index), 0);
			}
		}
	}

NoHit:

	// If we're only checking one submodel, return
	if (MC_struct_ptr->MC_info->flags & MC_SUBMODEL)	{
		return;
	}

	
	// If this subobject doesn't have any children, we're done checking it.
	if ( sm->num_children < 1 ) return;
	
	// Save instance (MC_struct_ptr->MC_orient, MC_struct_ptr->MC_base, Mc_point_base)
	matrix saved_orient = MC_struct_ptr->MC_orient;
	vec3d saved_base = MC_struct_ptr->MC_base;
	
	// Check all of this subobject's children
	i = sm->first_child;
	while ( i >= 0 )	{
		angles angs;
		bool blown_off;
		bool collision_checked;
		bsp_info * csm = &MC_struct_ptr->MC_polymodel->submodel[i];
		
		if ( MC_struct_ptr->MC_pmi ) {
			angs = MC_struct_ptr->MC_pmi->submodel[i].angs;
			blown_off = MC_struct_ptr->MC_pmi->submodel[i].blown_off;
			collision_checked = MC_struct_ptr->MC_pmi->submodel[i].collision_checked;
		} else {
			angs = csm->angs;
			blown_off = csm->blown_off ? true : false;
			collision_checked = false;
		}

		// Don't check it or its children if it is destroyed
		// or if it's set to no collision
		if ( !blown_off && !collision_checked && !csm->no_collisions )	{
			if ( MC_struct_ptr->MC_pmi ) {
				MC_struct_ptr->MC_orient = MC_struct_ptr->MC_pmi->submodel[i].mc_orient;
				MC_struct_ptr->MC_base = MC_struct_ptr->MC_pmi->submodel[i].mc_base;
				vm_vec_add2(&MC_struct_ptr->MC_base, MC_struct_ptr->MC_info->pos);
			} else {
				//instance for this subobject
				matrix tm = IDENTITY_MATRIX;

				vm_vec_unrotate(&MC_struct_ptr->MC_base, &csm->offset, &saved_orient );
				vm_vec_add2(&MC_struct_ptr->MC_base, &saved_base );

				if( vm_matrix_same(&tm, &csm->orientation)) {
					// if submodel orientation matrix is identity matrix then don't bother with matrix ops
					vm_angles_2_matrix(&tm, &angs);
				} else {
					matrix rotation_matrix = csm->orientation;
					vm_rotate_matrix_by_angles(&rotation_matrix, &angs);

					matrix inv_orientation;
					vm_copy_transpose_matrix(&inv_orientation, &csm->orientation);

					vm_matrix_x_matrix(&tm, &rotation_matrix, &inv_orientation);
				}

				vm_matrix_x_matrix(&MC_struct_ptr->MC_orient, &saved_orient, &tm);
			}

			mc_check_subobj( MC_struct_ptr, i );
		}

		i = csm->next_sibling;
	}

}

MONITOR(NumFVI)

// See model.h for usage.   I don't want to put the
// usage here because you need to see the #defines and structures
// this uses while reading the help.   
int model_collide(mc_info * model_collision_info)
{
	model_collision MC_struct;
	model_collision * MC_struct_ptr = &MC_struct;
	MONITOR_INC(NumFVI,1);

	MC_struct_ptr->MC_info = model_collision_info;
	MC_struct_ptr->MC_info->num_hits = 0;				// How many collisions were found
	MC_struct_ptr->MC_info->shield_hit_tri = -1;	// Assume we won't hit any shield polygons
	MC_struct_ptr->MC_info->hit_bitmap = -1;
	MC_struct_ptr->MC_info->edge_hit = 0;

	if ( (MC_struct_ptr->MC_info->flags & MC_CHECK_SHIELD) && (MC_struct_ptr->MC_info->flags & MC_CHECK_MODEL) )	{
		Error( LOCATION, "Checking both shield and model!\n" );
		return 0;
	}

	//Fill in some global variables that all the model collide routines need internally.
	MC_struct_ptr->MC_polymodel = model_get(MC_struct_ptr->MC_info->model_num);
	MC_struct_ptr->MC_orient = *MC_struct_ptr->MC_info->orient;
	MC_struct_ptr->MC_base = *MC_struct_ptr->MC_info->pos;
	MC_struct_ptr->MC_mag = vm_vec_dist( MC_struct_ptr->MC_info->p0, MC_struct_ptr->MC_info->p1 );

	if ( MC_struct_ptr->MC_info->model_instance_num >= 0 ) {
		MC_struct_ptr->MC_pmi = model_get_instance(MC_struct_ptr->MC_info->model_instance_num);
	} else {
		MC_struct_ptr->MC_pmi = NULL;
	}

	// DA 11/19/98 - disable this check for rotating submodels
	// Don't do check if for very small movement
//	if (MC_struct_ptr->MC_mag < 0.01f) {
//		return 0;
//	}

	float model_radius;		// How big is the model we're checking against
	int first_submodel;		// Which submodel gets returned as hit if MC_ONLY_SPHERE specified

	if ( (MC_struct_ptr->MC_info->flags & MC_SUBMODEL) || (MC_struct_ptr->MC_info->flags & MC_SUBMODEL_INSTANCE) )	{
		first_submodel = MC_struct_ptr->MC_info->submodel_num;
		model_radius = MC_struct_ptr->MC_polymodel->submodel[first_submodel].rad;
	} else {
		first_submodel = MC_struct_ptr->MC_polymodel->detail[0];
		model_radius = MC_struct_ptr->MC_polymodel->rad;
	}

	if ( MC_struct_ptr->MC_info->flags & MC_CHECK_SPHERELINE ) {
		if ( MC_struct_ptr->MC_info->radius <= 0.0f ) {
			Warning(LOCATION, "Attempting to collide with a sphere, but the sphere's radius is <= 0.0f!\n\n(model file is %s; submodel is %d, mc_flags are %d)", MC_struct_ptr->MC_polymodel->filename, first_submodel, MC_struct_ptr->MC_info->flags);
			return 0;
		}

		// Do a quick check on the Bounding Sphere
		if (fvi_segment_sphere(&MC_struct_ptr->MC_info->hit_point_world, MC_struct_ptr->MC_info->p0, MC_struct_ptr->MC_info->p1, MC_struct_ptr->MC_info->pos, model_radius+MC_struct_ptr->MC_info->radius) )	{
			if ( MC_struct_ptr->MC_info->flags & MC_ONLY_SPHERE )	{
				MC_struct_ptr->MC_info->hit_point = MC_struct_ptr->MC_info->hit_point_world;
				MC_struct_ptr->MC_info->hit_submodel = first_submodel;
				MC_struct_ptr->MC_info->num_hits++;
				return (MC_struct_ptr->MC_info->num_hits > 0);
			}
			// continue checking polygons.
		} else {
			return 0;
		}
	} else {
		int r;

		// Do a quick check on the Bounding Sphere
		if ( MC_struct_ptr->MC_info->flags & MC_CHECK_RAY ) {
			r = fvi_ray_sphere(&MC_struct_ptr->MC_info->hit_point_world, MC_struct_ptr->MC_info->p0, MC_struct_ptr->MC_info->p1, MC_struct_ptr->MC_info->pos, model_radius);
		} else {
			r = fvi_segment_sphere(&MC_struct_ptr->MC_info->hit_point_world, MC_struct_ptr->MC_info->p0, MC_struct_ptr->MC_info->p1, MC_struct_ptr->MC_info->pos, model_radius);
		}
		if (r) {
			if ( MC_struct_ptr->MC_info->flags & MC_ONLY_SPHERE ) {
				MC_struct_ptr->MC_info->hit_point = MC_struct_ptr->MC_info->hit_point_world;
				MC_struct_ptr->MC_info->hit_submodel = first_submodel;
				MC_struct_ptr->MC_info->num_hits++;
				return (MC_struct_ptr->MC_info->num_hits > 0);
			}
			// continue checking polygons.
		} else {
			return 0;
		}

	}

	if ( MC_struct_ptr->MC_info->flags & MC_SUBMODEL )	{
		// Check only one subobject
		mc_check_subobj( MC_struct_ptr, MC_struct_ptr->MC_info->submodel_num );
		// Check submodel and any children
	} else if (MC_struct_ptr->MC_info->flags & MC_SUBMODEL_INSTANCE) {
		mc_check_subobj(MC_struct_ptr, MC_struct_ptr->MC_info->submodel_num);
	} else {
		// Check all the the highest detail model polygons and subobjects for intersections

		// Don't check it or its children if it is destroyed
		if (!MC_struct_ptr->MC_polymodel->submodel[MC_struct_ptr->MC_polymodel->detail[0]].blown_off)	{
			mc_check_subobj( MC_struct_ptr, MC_struct_ptr->MC_polymodel->detail[0] );
		}
	}


	//If we found a hit, then rotate it into world coordinates	
	if ( MC_struct_ptr->MC_info->num_hits )	{
		if ( MC_struct_ptr->MC_info->flags & MC_SUBMODEL )	{
			// If we're just checking one submodel, don't use normal instancing to find world points
			vm_vec_unrotate(&MC_struct_ptr->MC_info->hit_point_world, &MC_struct_ptr->MC_info->hit_point, MC_struct_ptr->MC_info->orient);
			vm_vec_add2(&MC_struct_ptr->MC_info->hit_point_world, MC_struct_ptr->MC_info->pos);
		} else {
			if ( MC_struct_ptr->MC_pmi ) {
				model_instance_find_world_point(&MC_struct_ptr->MC_info->hit_point_world, &MC_struct_ptr->MC_info->hit_point, MC_struct_ptr->MC_info->model_num, MC_struct_ptr->MC_info->model_instance_num, MC_struct_ptr->MC_info->hit_submodel, MC_struct_ptr->MC_info->orient, MC_struct_ptr->MC_info->pos);
			} else {
				model_find_world_point(&MC_struct_ptr->MC_info->hit_point_world, &MC_struct_ptr->MC_info->hit_point, MC_struct_ptr->MC_info->model_num, MC_struct_ptr->MC_info->hit_submodel, MC_struct_ptr->MC_info->orient, MC_struct_ptr->MC_info->pos);
			}
		}
	}


	return MC_struct_ptr->MC_info->num_hits;

}

void model_collide_preprocess_subobj(vec3d *pos, matrix *orient, polymodel *pm,  polymodel_instance *pmi, int subobj_num)
{
	submodel_instance *smi = &pmi->submodel[subobj_num];

	smi->mc_base = *pos;
	smi->mc_orient = *orient;

	int i = pm->submodel[subobj_num].first_child;

	while ( i >= 0 ) {
		angles angs = pmi->submodel[i].angs;
		bsp_info * csm = &pm->submodel[i];

		matrix tm = IDENTITY_MATRIX;

		vm_vec_unrotate(pos, &csm->offset, &smi->mc_orient );
		vm_vec_add2(pos, &smi->mc_base);

		if( vm_matrix_same(&tm, &csm->orientation)) {
			// if submodel orientation matrix is identity matrix then don't bother with matrix ops
			vm_angles_2_matrix(&tm, &angs);
		} else {
			matrix rotation_matrix = csm->orientation;
			vm_rotate_matrix_by_angles(&rotation_matrix, &angs);

			matrix inv_orientation;
			vm_copy_transpose_matrix(&inv_orientation, &csm->orientation);

			vm_matrix_x_matrix(&tm, &rotation_matrix, &inv_orientation);
		}

		vm_matrix_x_matrix(orient, &smi->mc_orient, &tm);

		model_collide_preprocess_subobj(pos, orient, pm, pmi, i);

		i = csm->next_sibling;
	}
}

void model_collide_preprocess(matrix *orient, int model_instance_num)
{
	polymodel_instance	*pmi;
	polymodel *pm;

	pmi = model_get_instance(model_instance_num);
	pm = model_get(pmi->model_num);

	matrix current_orient = *orient;
	vec3d current_pos;

	vm_vec_zero(&current_pos);

	model_collide_preprocess_subobj(&current_pos, &current_orient, pm, pmi, pm->detail[0]);
}
