/*
 * Copyright (C) Volition, Inc. 1999.  All rights reserved.
 *
 * All source code herein is the property of Volition, Inc. You may not sell 
 * or otherwise commercially exploit the source or things you created based on the 
 * source.
 *
*/




#ifndef __BM_INTERNAL_H__
#define __BM_INTERNAL_H__


// extra check to make sure this stuff doesn't end up in normal files
// don't use any of this unless BMPMAN_INTERNAL is defined
#ifdef BMPMAN_INTERNAL

#include "globalincs/pstypes.h"
#include "bmpman/bmpman.h"

#include "ddsutils/ddsutils.h"
#include "pcxutils/pcxutils.h"
#include "pngutils/pngutils.h"
#include "jpgutils/jpgutils.h"
#include "tgautils/tgautils.h"
#include "palman/palman.h"

// no-type			( used in: bm_bitmaps[i].type )
#define BM_TYPE_NONE			0
// in-memory type	( used in: bm_bitmaps[i].type )
#define BM_TYPE_USER			1
// file-type types	( used in: bm_bitmaps[i].type )
#define BM_TYPE_PCX				2
#define BM_TYPE_TGA				3		// 16 or 32 bit targa
#define BM_TYPE_DDS				4		// generic identifier for DDS
#define BM_TYPE_PNG				5		// PNG
#define BM_TYPE_JPG				6		// 32 bit jpeg
#define BM_TYPE_ANI				7		// in-house ANI format
#define BM_TYPE_EFF				8		// specifies any type of animated image, the EFF itself is just text
// c-type types		( used in: bm_bitmaps[i].c_type )
#define BM_TYPE_DXT1			9		// 24 bit with switchable alpha		(compressed)
#define BM_TYPE_DXT3			10		// 32 bit with 4 bit alpha			(compressed)
#define BM_TYPE_DXT5			11		// 32 bit with 8 bit alpha			(compressed)
#define BM_TYPE_CUBEMAP_DDS		12		// generic DDS cubemap	(uncompressed cubemap surface)
#define BM_TYPE_CUBEMAP_DXT1	13		// 24-bit cubemap		(compressed cubemap surface)
#define BM_TYPE_CUBEMAP_DXT3	14		// 32-bit cubemap		(compressed cubemap surface)
#define BM_TYPE_CUBEMAP_DXT5	15		// 32-bit cubemap		(compressed cubemap surface)
// special types	( used in: bm_bitmaps[i].type )
#define BM_TYPE_RENDER_TARGET_STATIC	16		// 24/32 bit setup internally as a static render target
#define BM_TYPE_RENDER_TARGET_DYNAMIC	17		// 24/32 bit setup internally as a dynamic render target


/// Moved from cpp file ///////////////////
// Consider these 'protected' structures and functions that should only be used by special bitmap functions
typedef union bm_extra_info {
	struct {
		// Stuff needed for animations
		int		first_frame;								// used for animations -- points to index of first frame
		int		num_frames;									// used for animation -- number of frames in the animation
		int		keyframe;									// used for animation -- keyframe info
		ubyte	fps;										// used for animation -- frames per second

		struct {
			// stuff for static animations
			ubyte	type;									// type for individual images
			char	filename[MAX_FILENAME_LEN];				// filename for individual images
		} eff;
	} ani;

	struct {
		// Stuff needed for user bitmaps
		void		*data;									// For user bitmaps, this is where the data comes from
		ubyte		bpp;									// For user bitmaps, this is what format the data is
		ubyte		flags;									// Flags passed to bm_create
	} user;
} bm_extra_info;


typedef struct bitmap_entry	{
	// identification
	char		filename[MAX_FILENAME_LEN];			// filename for this bitmap

	uint		signature;									// a unique signature identifying the data
	uint		palette_checksum;							// checksum used to be sure bitmap is in current palette
	int		handle;										// Handle = id*MAX_BITMAPS + bitmapnum
	int		last_used;									// When this bitmap was last used

	ubyte		type;									// PCX, USER, ANI, etc
	ubyte		comp_type;								// What sort of compressed type, BM_TYPE_NONE if not compressed
	signed char	ref_count;								// Number of locks on bitmap.  Can't unload unless ref_count is 0.

	int		dir_type;								// which directory this was loaded from (to skip other locations with same name)

	// compressed bitmap stuff (.dds) - RT please take a look at this and tell me if we really need it
	int		mem_taken;									// How much memory does this bitmap use? - UnknownPlayer
	int		num_mipmaps;								// number of mipmap levels, we need to read all of them

	// Stuff to keep track of usage
	ubyte		preloaded;									// If set, then this was loaded from the lst file
	int			preload_count;								// how many times this gets used in game, for unlocking
	ubyte		used_flags;									// What flags it was accessed thru
	int			load_count;

	// Bitmap info
	bitmap	bm;

	// Data for animations and user bitmaps
	bm_extra_info	info;		

#ifdef BMPMAN_NDEBUG
	// bookeeping
	ubyte		used_last_frame;							// If set, then it was used last frame
	ubyte		used_this_frame;							// If set, then it was used this frame
	int		data_size;									// How much data this bitmap uses
	int		used_count;									// How many times it was accessed
#endif
} bitmap_entry;

extern bitmap_entry bm_bitmaps[MAX_BITMAPS];
extern int Is_standalone;
//
//// image specific lock functions
//void bm_lock_ani( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags );
//void bm_lock_dds( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags );
//void bm_lock_png( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags );
//void bm_lock_jpg( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags );
//void bm_lock_pcx( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags );
//void bm_lock_tga( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags );
//void bm_lock_user( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags );
#define EFF_FILENAME_CHECK { if ( be->type == BM_TYPE_EFF ) strncpy( filename, be->info.ani.eff.filename, MAX_FILENAME_LEN ); else strncpy( filename, be->filename, MAX_FILENAME_LEN ); }

inline void bm_lock_pcx( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags )
{
  ubyte *data;
  int pcx_error;
  char filename[MAX_FILENAME_LEN];

  // Unload any existing data
  bm_free_data( bitmapnum, false );

  be->mem_taken = (bmp->w * bmp->h * (bpp >> 3));
  data = (ubyte *)bm_malloc(bitmapnum, be->mem_taken);
  bmp->bpp = bpp;
  bmp->data = (ptr_u)data;
  bmp->palette = (bpp == 8) ? gr_palette : NULL;
  memset( data, 0, be->mem_taken );

  Assert( &be->bm == bmp );
#ifdef BMPMAN_NDEBUG
  Assert( be->data_size > 0 );
#endif

  // some sanity checks on flags
  Assert(!((flags & BMP_AABITMAP) && (flags & BMP_TEX_ANY)));           // no aabitmap textures

  // make sure we are using the correct filename in the case of an EFF.
  // this will populate filename[] whether it's EFF or not
  EFF_FILENAME_CHECK;

  pcx_error = pcx_read_bitmap( filename, data, NULL, (bpp >> 3), (flags & BMP_AABITMAP), 0, be->dir_type );

  if ( pcx_error != PCX_ERROR_NONE ) {
    mprintf(("Couldn't load PCX!!! (%s)\n", filename));
    return;
  }

#ifdef BMPMAN_NDEBUG
  Assert( be->data_size > 0 );
#endif

  bmp->flags = 0;

  bm_convert_format( bitmapnum, bmp, bpp, flags );
}

inline void bm_lock_ani( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags )
{
  anim        *the_anim;
  anim_instance *the_anim_instance;
  bitmap      *bm;
  ubyte       *frame_data;
  int       size, i;
  int       first_frame, nframes;

  first_frame = be->info.ani.first_frame;
  nframes = bm_bitmaps[first_frame].info.ani.num_frames;

  if ( (the_anim = anim_load(bm_bitmaps[first_frame].filename, bm_bitmaps[first_frame].dir_type)) == NULL ) {
    nprintf(("BMPMAN", "Error opening %s in bm_lock\n", be->filename));
    return;
  }

  if ( (the_anim_instance = init_anim_instance(the_anim, bpp)) == NULL ) {
    nprintf(("BMPMAN", "Error opening %s in bm_lock\n", be->filename));
    anim_free(the_anim);
    return;
  }

  int can_drop_frames = 0;

  if ( the_anim->total_frames != bm_bitmaps[first_frame].info.ani.num_frames )  {
    can_drop_frames = 1;
  }

  bm = &bm_bitmaps[first_frame].bm;
  size = bm->w * bm->h * (bpp >> 3);
  be->mem_taken = size;

  Assert( size > 0 );

  for ( i=0; i<nframes; i++ ) {
    be = &bm_bitmaps[first_frame+i];
    bm = &bm_bitmaps[first_frame+i].bm;

    // Unload any existing data
    bm_free_data( first_frame+i, false );

    bm->flags = 0;

    // briefing editor in Fred2 uses aabitmaps (ani's) - force to 8 bit
    bm->bpp = Is_standalone ? (ubyte)8 : bpp;

    bm->data = (ptr_u)bm_malloc(first_frame + i, size);

    frame_data = anim_get_next_raw_buffer(the_anim_instance, 0 ,flags & BMP_AABITMAP ? 1 : 0, bm->bpp);

    ubyte *dptr, *sptr;

    sptr = frame_data;
    dptr = (ubyte *)bm->data;

    if ( (bm->w!=the_anim->width) || (bm->h!=the_anim->height) )  {
      // Scale it down
      // 8 bit
      if(bpp == 8){
        int w,h;
        fix u, utmp, v, du, dv;

        u = v = 0;

        du = ( the_anim->width*F1_0 ) / bm->w;
        dv = ( the_anim->height*F1_0 ) / bm->h;

        for (h = 0; h < bm->h; h++) {
          ubyte *drow = &dptr[bm->w * h];
          ubyte *srow = &sptr[f2i(v)*the_anim->width];

          utmp = u;

          for (w = 0; w < bm->w; w++) {
            *drow++ = srow[f2i(utmp)];
            utmp += du;
          }
          v += dv;
        }
      }
      // 16 bpp
      else {
        int w,h;
        fix u, utmp, v, du, dv;

        u = v = 0;

        du = ( the_anim->width*F1_0 ) / bm->w;
        dv = ( the_anim->height*F1_0 ) / bm->h;

        for (h = 0; h < bm->h; h++) {
          unsigned short *drow = &((unsigned short*)dptr)[bm->w * h];
          unsigned short *srow = &((unsigned short*)sptr)[f2i(v)*the_anim->width];

          utmp = u;

          for (w = 0; w < bm->w; w++) {
            *drow++ = srow[f2i(utmp)];
            utmp += du;
          }
          v += dv;
        }
      }
    } else {
      // 1-to-1 mapping
      memcpy(dptr, sptr, size);
    }

    bm_convert_format( first_frame+i, bm, bpp, flags );

    // Skip a frame
    if ( (i < nframes-1)  && can_drop_frames )  {
      frame_data = anim_get_next_raw_buffer(the_anim_instance, 0, flags & BMP_AABITMAP ? 1 : 0, bm->bpp);
    }
  }

  free_anim_instance(the_anim_instance);
  anim_free(the_anim);
}


inline void bm_lock_user( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags )
{
  // Unload any existing data
  bm_free_data( bitmapnum, false );

  if ((bpp != be->info.user.bpp) && !(flags & BMP_AABITMAP))
    bpp = be->info.user.bpp;

  switch ( bpp ) {
    case 32:  // user 32-bit bitmap
      bmp->bpp = bpp;
      bmp->flags = be->info.user.flags;
      bmp->data = (ptr_u)be->info.user.data;
      break;

    case 24:  // user 24-bit bitmap
      bmp->bpp = bpp;
      bmp->flags = be->info.user.flags;
      bmp->data = (ptr_u)be->info.user.data;
      break;

    case 16:      // user 16 bit bitmap
      bmp->bpp = bpp;
      bmp->flags = be->info.user.flags;
      bmp->data = (ptr_u)be->info.user.data;
      break;

    case 8:     // Going from 8 bpp to something (probably only for aabitmaps)
      Assert(flags & BMP_AABITMAP);
      bmp->bpp = bpp;
      bmp->flags = be->info.user.flags;
      bmp->data = (ptr_u)be->info.user.data;
      break;

     default:
      Error( LOCATION, "Unhandled user bitmap conversion from %d to %d bpp", be->info.user.bpp, bmp->bpp );
      break;
  }

  bm_convert_format( bitmapnum, bmp, bpp, flags );
}

inline void bm_lock_tga( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags )
{
  ubyte *data = NULL;
  int d_size, byte_size;
  char filename[MAX_FILENAME_LEN];

  // Unload any existing data
  bm_free_data( bitmapnum, false );

  if(Is_standalone){
    Assert(bpp == 8);
  }
  else
  {
    Assert( (bpp == 16) || (bpp == 24 ) || (bpp == 32) );
  }

  // allocate bitmap data
  byte_size = (bpp >> 3);

  Assert( byte_size );
  Assert( be->mem_taken > 0 );

  data = (ubyte*)bm_malloc(bitmapnum, be->mem_taken);

  if (data) {
    memset( data, 0, be->mem_taken);
    d_size = byte_size;
  } else {
    return;
  }

  bmp->bpp = bpp;
  bmp->data = (ptr_u)data;
  bmp->palette = NULL;

  Assert( &be->bm == bmp );
#ifdef BMPMAN_NDEBUG
  Assert( be->data_size > 0 );
#endif

  int tga_error;

  // make sure we are using the correct filename in the case of an EFF.
  // this will populate filename[] whether it's EFF or not
  EFF_FILENAME_CHECK;

  tga_error = targa_read_bitmap( filename, data, NULL, d_size, be->dir_type);

  if ( tga_error != TARGA_ERROR_NONE )  {
    bm_free_data( bitmapnum, false );
    return;
  }

  bmp->flags = 0;

  bm_convert_format( bitmapnum, bmp, bpp, flags );
}

/**
 * Lock a DDS file
 */
inline void bm_lock_dds( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags )
{
  ubyte *data = NULL;
  int error;
  ubyte dds_bpp = 0;
  char filename[MAX_FILENAME_LEN];

  // free any existing data
  bm_free_data( bitmapnum, false );

  Assert( be->mem_taken > 0 );
  Assert( &be->bm == bmp );

  data = (ubyte*)bm_malloc(bitmapnum, be->mem_taken);

  if ( data == NULL )
    return;

  memset( data, 0, be->mem_taken );

  // make sure we are using the correct filename in the case of an EFF.
  // this will populate filename[] whether it's EFF or not
  EFF_FILENAME_CHECK;

  error = dds_read_bitmap( filename, data, &dds_bpp, be->dir_type );

#if BYTE_ORDER == BIG_ENDIAN
  // same as with TGA, we need to byte swap 16 & 32-bit, uncompressed, DDS images
  if ( (be->comp_type == BM_TYPE_DDS) || (be->comp_type == BM_TYPE_CUBEMAP_DDS) ) {
    unsigned int i = 0;

    if (dds_bpp == 32) {
      unsigned int *swap_tmp;

      for (i = 0; i < (unsigned int)be->mem_taken; i += 4) {
        swap_tmp = (unsigned int *)(data + i);
        *swap_tmp = INTEL_INT(*swap_tmp);
      }
    } else if (dds_bpp == 16) {
      unsigned short *swap_tmp;

      for (i = 0; i < (unsigned int)be->mem_taken; i += 2) {
        swap_tmp = (unsigned short *)(data + i);
        *swap_tmp = INTEL_SHORT(*swap_tmp);
      }
    }
  }
#endif

  bmp->bpp = dds_bpp;
  bmp->data = (ptr_u)data;
  bmp->flags = 0;

  if (error != DDS_ERROR_NONE) {
    bm_free_data( bitmapnum, false );
    return;
  }

#ifdef BMPMAN_NDEBUG
  Assert( be->data_size > 0 );
#endif
}

/**
 * Lock a PNG file
 */
inline void bm_lock_png( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags )
{
  ubyte *data = NULL;
  //assume 32 bit - libpng should expand everything
  int d_size;
  int png_error = PNG_ERROR_INVALID;
  char filename[MAX_FILENAME_LEN];

  // Unload any existing data
  bm_free_data( bitmapnum, false );

  // allocate bitmap data
  Assert( bmp->w * bmp->h > 0 );

  //if it's not 32-bit, we expand when we read it
  bmp->bpp = 32;
  d_size = bmp->bpp >> 3;
  //we waste memory if it turns out to be 24-bit, but the way this whole thing works is dodgy anyway
  data = (ubyte*)bm_malloc(bitmapnum, bmp->w * bmp->h * d_size);
  if (data == NULL)
    return;
  memset( data, 0, bmp->w * bmp->h * d_size);
  bmp->data = (ptr_u)data;
  bmp->palette = NULL;

  Assert( &be->bm == bmp );

  // make sure we are using the correct filename in the case of an EFF.
  // this will populate filename[] whether it's EFF or not
  EFF_FILENAME_CHECK;

  //bmp->bpp gets set correctly in here after reading into memory
  png_error = png_read_bitmap( filename, data, &bmp->bpp, d_size, be->dir_type );

  if ( png_error != PNG_ERROR_NONE )  {
    bm_free_data( bitmapnum, false );
    return;
  }

#ifdef BMPMAN_NDEBUG
  Assert( be->data_size > 0 );
#endif
}

/**
 * Lock a JPEG file
 */
inline void bm_lock_jpg( int handle, int bitmapnum, bitmap_entry *be, bitmap *bmp, ubyte bpp, ubyte flags )
{
  ubyte *data = NULL;
  int d_size = 0;
  int jpg_error = JPEG_ERROR_INVALID;
  char filename[MAX_FILENAME_LEN];

  // Unload any existing data
  bm_free_data( bitmapnum, false );

  d_size = (bpp >> 3);

  // allocate bitmap data
  Assert( be->mem_taken > 0 );
  data = (ubyte*)bm_malloc(bitmapnum, be->mem_taken);

  if (data == NULL)
    return;

  memset( data, 0, be->mem_taken);

  bmp->bpp = bpp;
  bmp->data = (ptr_u)data;
  bmp->palette = NULL;

  Assert( &be->bm == bmp );

  // make sure we are using the correct filename in the case of an EFF.
  // this will populate filename[] whether it's EFF or not
  EFF_FILENAME_CHECK;

  jpg_error = jpeg_read_bitmap( filename, data, NULL, d_size, be->dir_type );

  if ( jpg_error != JPEG_ERROR_NONE ) {
    bm_free_data( bitmapnum, false );
    return;
  }

#ifdef BMPMAN_NDEBUG
  Assert( be->data_size > 0 );
#endif
}

#endif // BMPMAN_INTERNAL

#endif // __BM_INTERNAL_H__
