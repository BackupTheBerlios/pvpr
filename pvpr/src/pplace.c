#include <stdio.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include "util.h"
#include "barrier.h"
#include "vpr_types.h"
#include "globals.h"
#include "pplace.h"
#include "place.h"
#include "read_place.h"
#include "draw.h"
#include "place_and_route.h"
#include "net_delay.h"
#include "path_delay.h"
#include "timing_place_lookup.h"
#include "timing_place.h"

#define FROM 0      /* What block connected to a net has moved? */
#define TO 1
#define FROM_AND_TO 2
#define EMPTY -1
#define SMALL_NET 4
#define ERROR_TOL .001
#define MAX_MOVES_BEFORE_RECOMPUTE 1000000

static float recompute_bb_cost (struct pcontext *, int, int);
static int ptry_swap (struct pcontext *context);
static void find_to (struct pcontext *context, int x_from, int y_from, int type, float rlim, int *x_to, int *y_to);
static int find_affected_nets (struct pcontext *context, int b_from, int b_to, int num_of_pins);
static void get_non_updateable_bb (struct pcontext *context, int inet, struct s_bb *bb_coord_new);
static void update_bb (struct pcontext *context, int inet, struct s_bb *bb_coord_new, struct s_bb *bb_edge_new, int xold, int yold, int xnew, int ynew);
static void get_bb_from_scratch (struct pcontext *context, int inet, struct s_bb *coords, struct s_bb *num_on_edges);
static int assess_swap (struct pcontext *context, float delta_c, float t);

static int assess_swap (struct pcontext *context, float delta_c, float t) {

/* Returns: 1 -> move accepted, 0 -> rejected. */ 

	int accept;
	float prob_fac, fnum;

	if (delta_c <= 0) {

#ifdef SPEC          /* Reduce variation in final solution due to round off */
    fnum = p_my_frand();
#endif

		accept = 1;
		return(accept);
	}

	if (t == 0.) 
		return(0);

	fnum = p_my_frand(context);
	prob_fac = exp(-delta_c/t);
	if (prob_fac > fnum) {
		accept = 1;
	}
	else {
		accept = 0;
	}
	return(accept);
}

static void get_bb_from_scratch (struct pcontext *context, int inet, struct s_bb *coords, struct s_bb *num_on_edges) {

/* This routine finds the bounding box of each net from scratch (i.e.    *
 * from only the block location information).  It updates both the       *
 * coordinate and number of blocks on each edge information.  It         *
 * should only be called when the bounding box information is not valid. */

	int ipin, bnum, x, y, xmin, xmax, ymin, ymax;
	int xmin_edge, xmax_edge, ymin_edge, ymax_edge;
	int n_pins;
	int *plist;

/* I need a list of blocks to which this net connects, with no block listed *
 * more than once, in order to get a proper count of the number on the edge *
 * of the bounding box.                                                     */

	if (context->duplicate_pins[inet] == 0) {
		plist = net[inet].blocks;
		n_pins = net[inet].num_pins;
	}
	else {
		plist = context->unique_pin_list[inet];
		n_pins = net[inet].num_pins - context->duplicate_pins[inet];
	}

	x = context->block[plist[0]].x;
	y = context->block[plist[0]].y;

	x = max(min(x,nx),1);   
	y = max(min(y,ny),1);

	xmin = x;
	ymin = y;
	xmax = x;
	ymax = y;
	xmin_edge = 1;
	ymin_edge = 1;
	xmax_edge = 1;
	ymax_edge = 1;
 
	for (ipin=1;ipin<n_pins;ipin++) {
 
		bnum = plist[ipin];
		x = context->block[bnum].x;
		y = context->block[bnum].y;

/* Code below counts IO blocks as being within the 1..nx, 1..ny clb array. *
 * This is because channels do not go out of the 0..nx, 0..ny range, and   *
 * I always take all channels impinging on the bounding box to be within   *
 * that bounding box.  Hence, this "movement" of IO blocks does not affect *
 * the which channels are included within the bounding box, and it         *
 * simplifies the code a lot.                                              */

		x = max(min(x,nx),1);   
		y = max(min(y,ny),1);

		if (x == xmin) {  
			xmin_edge++;
		}
		if (x == xmax) {  /* Recall that xmin could equal xmax -- don't use else */
			xmax_edge++;
		}
		else if (x < xmin) {
			xmin = x;
			xmin_edge = 1;
		}
		else if (x > xmax) {
			xmax = x;
			xmax_edge = 1;
		}

		if (y == ymin) {
			ymin_edge++;
		}
		if (y == ymax) {
			ymax_edge++;
		}
		else if (y < ymin) {
			ymin = y;
			ymin_edge = 1;
		}
		else if (y > ymax) {
			ymax = y;
			ymax_edge = 1;
		}
	}

/* Copy the coordinates and number on edges information into the proper   *
 * structures.                                                            */

	coords->xmin = xmin;
	coords->xmax = xmax;
	coords->ymin = ymin;
	coords->ymax = ymax;

	num_on_edges->xmin = xmin_edge;
	num_on_edges->xmax = xmax_edge;
	num_on_edges->ymin = ymin_edge;
	num_on_edges->ymax = ymax_edge;
}

static void update_bb (struct pcontext *context, int inet, struct s_bb *bb_coord_new, struct s_bb *bb_edge_new, int xold, int yold, int xnew, int ynew) {

/* Updates the bounding box of a net by storing its coordinates in    *
 * the bb_coord_new data structure and the number of blocks on each   *
 * edge in the bb_edge_new data structure.  This routine should only  *
 * be called for large nets, since it has some overhead relative to   *
 * just doing a brute force bounding box calculation.  The bounding   *
 * box coordinate and edge information for inet must be valid before  *
 * this routine is called.                                            *
 * Currently assumes channels on both sides of the CLBs forming the   *
 * edges of the bounding box can be used.  Essentially, I am assuming *
 * the pins always lie on the outside of the bounding box.            */

/* IO blocks are considered to be one cell in for simplicity. */

	xnew = max(min(xnew,nx),1);
	ynew = max(min(ynew,ny),1);
	xold = max(min(xold,nx),1);
	yold = max(min(yold,ny),1);

/* Check if I can update the bounding box incrementally. */ 

	if (xnew < xold) {                          /* Move to left. */

/* Update the xmax fields for coordinates and number of edges first. */

		if (xold == context->bb_coords[inet].xmax) {     /* Old position at xmax. */
				if (context->bb_num_on_edges[inet].xmax == 1) {
					get_bb_from_scratch (context, inet, bb_coord_new, bb_edge_new);
					return;
				}
				else {
					bb_edge_new->xmax = context->bb_num_on_edges[inet].xmax - 1;
					bb_coord_new->xmax = context->bb_coords[inet].xmax; 
				}
		}
		else {              /* Move to left, old postion was not at xmax. */
			bb_coord_new->xmax = context->bb_coords[inet].xmax; 
			bb_edge_new->xmax = context->bb_num_on_edges[inet].xmax;
		}

/* Now do the xmin fields for coordinates and number of edges. */

		if (xnew < context->bb_coords[inet].xmin) {    /* Moved past xmin */
			bb_coord_new->xmin = xnew;
			bb_edge_new->xmin = 1;
		}
		else if (xnew == context->bb_coords[inet].xmin) {   /* Moved to xmin */
			bb_coord_new->xmin = xnew;
			bb_edge_new->xmin = context->bb_num_on_edges[inet].xmin + 1;
		}
		else {                                  /* Xmin unchanged. */
			bb_coord_new->xmin = context->bb_coords[inet].xmin;
			bb_edge_new->xmin = context->bb_num_on_edges[inet].xmin;
		}
	}    /* End of move to left case. */
	else if (xnew > xold) {             /* Move to right. */
    
/* Update the xmin fields for coordinates and number of edges first. */

		if (xold == context->bb_coords[inet].xmin) {   /* Old position at xmin. */
			if (context->bb_num_on_edges[inet].xmin == 1) {
					get_bb_from_scratch (context, inet, bb_coord_new, bb_edge_new);
					return;
			}
			else {
				bb_edge_new->xmin = context->bb_num_on_edges[inet].xmin - 1;
				bb_coord_new->xmin = context->bb_coords[inet].xmin;
			}
		}
		else {                /* Move to right, old position was not at xmin. */
			bb_coord_new->xmin = context->bb_coords[inet].xmin;
			bb_edge_new->xmin = context->bb_num_on_edges[inet].xmin;
		}

/* Now do the xmax fields for coordinates and number of edges. */

		if (xnew > context->bb_coords[inet].xmax) {    /* Moved past xmax. */
			bb_coord_new->xmax = xnew;
			bb_edge_new->xmax = 1;   
		} 
		else if (xnew == context->bb_coords[inet].xmax) {   /* Moved to xmax */
			bb_coord_new->xmax = xnew;
			bb_edge_new->xmax = context->bb_num_on_edges[inet].xmax + 1;
		} 
		else {                                  /* Xmax unchanged. */ 
			bb_coord_new->xmax = context->bb_coords[inet].xmax; 
			bb_edge_new->xmax = context->bb_num_on_edges[inet].xmax;   
		}
	}    /* End of move to right case. */

	else {          /* xnew == xold -- no x motion. */
		bb_coord_new->xmin = context->bb_coords[inet].xmin;
		bb_coord_new->xmax = context->bb_coords[inet].xmax;
		bb_edge_new->xmin = context->bb_num_on_edges[inet].xmin;
		bb_edge_new->xmax = context->bb_num_on_edges[inet].xmax;
	}

/* Now account for the y-direction motion. */

	if (ynew < yold) {                  /* Move down. */

/* Update the ymax fields for coordinates and number of edges first. */

		if (yold == context->bb_coords[inet].ymax) {    /* Old position at ymax. */
			if (context->bb_num_on_edges[inet].ymax == 1) {
				get_bb_from_scratch (context, inet, bb_coord_new, bb_edge_new);
				return;
			}
			else {
				bb_edge_new->ymax = context->bb_num_on_edges[inet].ymax - 1;
				bb_coord_new->ymax = context->bb_coords[inet].ymax;
			}
		}     
		else {              /* Move down, old postion was not at ymax. */
			bb_coord_new->ymax = context->bb_coords[inet].ymax;
			bb_edge_new->ymax = context->bb_num_on_edges[inet].ymax;
		}     
 
/* Now do the ymin fields for coordinates and number of edges. */
 
		if (ynew < context->bb_coords[inet].ymin) {    /* Moved past ymin */
			bb_coord_new->ymin = ynew;
			bb_edge_new->ymin = 1;
		}     
		else if (ynew == context->bb_coords[inet].ymin) {   /* Moved to ymin */
			bb_coord_new->ymin = ynew;
			bb_edge_new->ymin = context->bb_num_on_edges[inet].ymin + 1;
		}     
		else {                                  /* ymin unchanged. */
			bb_coord_new->ymin = context->bb_coords[inet].ymin;
			bb_edge_new->ymin = context->bb_num_on_edges[inet].ymin;
		}     
	}    /* End of move down case. */
	else if (ynew > yold) {             /* Moved up. */
    
/* Update the ymin fields for coordinates and number of edges first. */
 
		if (yold == context->bb_coords[inet].ymin) {   /* Old position at ymin. */
			if (context->bb_num_on_edges[inet].ymin == 1) {
				get_bb_from_scratch (context, inet, bb_coord_new, bb_edge_new);
				return;
			}
			else {
				bb_edge_new->ymin = context->bb_num_on_edges[inet].ymin - 1;
				bb_coord_new->ymin = context->bb_coords[inet].ymin;
			}
		}     
		else {                /* Moved up, old position was not at ymin. */
			bb_coord_new->ymin = context->bb_coords[inet].ymin;
			bb_edge_new->ymin = context->bb_num_on_edges[inet].ymin;
		}     
 
/* Now do the ymax fields for coordinates and number of edges. */
 
		if (ynew > context->bb_coords[inet].ymax) {    /* Moved past ymax. */
			bb_coord_new->ymax = ynew;
			bb_edge_new->ymax = 1;
		}     
		else if (ynew == context->bb_coords[inet].ymax) {   /* Moved to ymax */
			bb_coord_new->ymax = ynew;
			bb_edge_new->ymax = context->bb_num_on_edges[inet].ymax + 1;
		}     
		else {                                  /* ymax unchanged. */
			bb_coord_new->ymax = context->bb_coords[inet].ymax;
			bb_edge_new->ymax = context->bb_num_on_edges[inet].ymax;
		}     
	}    /* End of move up case. */
	else {          /* ynew == yold -- no y motion. */
		bb_coord_new->ymin = context->bb_coords[inet].ymin;
		bb_coord_new->ymax = context->bb_coords[inet].ymax;
		bb_coord_new->ymin = context->bb_num_on_edges[inet].ymin;
		bb_coord_new->ymax = context->bb_num_on_edges[inet].ymax;
	}
}

static void get_non_updateable_bb (struct pcontext *context, int inet, struct s_bb *bb_coord_new) {

/* Finds the bounding box of a net and stores its coordinates in the  *
 * bb_coord_new data structure.  This routine should only be called   *
 * for small nets, since it does not determine enough information for *
 * the bounding box to be updated incrementally later.                *
 * Currently assumes channels on both sides of the CLBs forming the   *
 * edges of the bounding box can be used.  Essentially, I am assuming *
 * the pins always lie on the outside of the bounding box.            */


	int k, xmax, ymax, xmin, ymin, x, y;
 
	x = context->block[net[inet].blocks[0]].x;
	y = context->block[net[inet].blocks[0]].y;

	xmin = x;
	ymin = y;
	xmax = x;
	ymax = y;

	for (k=1;k<net[inet].num_pins;k++) {
		x = context->block[net[inet].blocks[k]].x;
		y = context->block[net[inet].blocks[k]].y;

		if (x < xmin) {
			xmin = x;
		}
		else if (x > xmax) {
			xmax = x;
		}

		if (y < ymin) {
			ymin = y;
		}
		else if (y > ymax ) {
			ymax = y;
		}
	}

 /* Now I've found the coordinates of the bounding box.  There are no *
  * channels beyond nx and ny, so I want to clip to that.  As well,   *
  * since I'll always include the channel immediately below and the   *
  * channel immediately to the left of the bounding box, I want to    *
  * clip to 1 in both directions as well (since minimum channel index *
  * is 0).  See route.c for a channel diagram.                        */
 
	bb_coord_new->xmin = max(min(xmin,nx),1);
	bb_coord_new->ymin = max(min(ymin,ny),1);
	bb_coord_new->xmax = max(min(xmax,nx),1);
	bb_coord_new->ymax = max(min(ymax,ny),1);
}

static int find_affected_nets (struct pcontext *context, int b_from, int b_to, int num_of_pins) {

/* Puts a list of all the nets connected to b_from and b_to into          *
 * nets_to_update.  Returns the number of affected nets.  Net_block_moved *
 * is either FROM, TO or FROM_AND_TO -- the block connected to this net   *
 * that has moved.                                                        */

	int k, inet, affected_index, count;

	affected_index = 0;

	for (k=0;k<num_of_pins;k++) {
		inet = context->block[b_from].nets[k];
   
		if (inet == OPEN) 
			continue;

		if (is_global[inet]) 
			continue;

/* This is here in case the same block connects to a net twice. */

		if (context->temp_net_cost[inet] > 0.)  
			continue;

		context->nets_to_update[affected_index] = inet;
		context->net_block_moved[affected_index] = FROM;
		affected_index++;
		context->temp_net_cost[inet] = 1.; /* Flag to say we've marked this net. */
	}

	if (b_to != EMPTY) {
		for (k=0;k<num_of_pins;k++) {
			inet = context->block[b_to].nets[k];
    
			if (inet == OPEN) 
				continue;
 
			if (is_global[inet]) 
				continue;

			if (context->temp_net_cost[inet] > 0.) {         /* Net already marked. */
				for (count=0;count<affected_index;count++) {
					if (context->nets_to_update[count] == inet) {
						if (context->net_block_moved[count] == FROM) 
							context->net_block_moved[count] = FROM_AND_TO;
						break;
					}
				}

#ifdef DEBUG
          if (count > affected_index) {
             printf("Error in find_affected_nets -- count = %d,"
              " affected index = %d.\n", count, affected_index);
             exit (1);
          }
#endif
			}
			else {           /* Net not marked yet. */

				context->nets_to_update[affected_index] = inet;
				context->net_block_moved[affected_index] = TO;
				affected_index++;
				context->temp_net_cost[inet] = 1.;  /* Flag means we've  marked net. */
			}
		}
	}
	return (affected_index);
}

static void find_to (struct pcontext *context, int x_from, int y_from, int type, float rlim, int *x_to, int *y_to) {

 /* Returns the point to which I want to swap, properly range limited. *
  * rlim must always be between 1 and nx (inclusive) for this routine  *
  * to work.                                                           */

	int x_rel, y_rel, iside, iplace, rlx, rly;

	rlx = min(nx,rlim);   /* Only needed when nx < ny. */
	rly = min (ny,rlim);  /* Added rly for aspect_ratio != 1 case. */

#ifdef DEBUG
 if (rlx < 1 || rlx > nx) {
    printf("Error in find_to: rlx = %d\n",rlx);
    exit(1);
 }
#endif

	do {              /* Until (x_to, y_to) different from (x_from, y_from) */
		if (type == CLB) {
			x_rel = p_my_irand (context, 2*rlx);    
			y_rel = p_my_irand (context, 2*rly);
			*x_to = x_from - rlx + x_rel;
			*y_to = y_from - rly + y_rel;
			if (*x_to > nx) *x_to = *x_to - nx;    /* better spectral props. */
			if (*x_to < 1) *x_to = *x_to + nx;     /* than simple min, max   */
			if (*y_to > ny) *y_to = *y_to - ny;    /* clipping.              */
			if (*y_to < 1) *y_to = *y_to + ny;
		}
		else {                 /* io_block to be moved. */
			if (rlx >= nx) {
				iside = p_my_irand(context,3);
/*                              *
 *       +-----1----+           *
 *       |          |           *
 *       |          |           *
 *       0          2           *
 *       |          |           *
 *       |          |           *
 *       +-----3----+           *
 *                              */
				switch (iside) {
					case 0:
						iplace = p_my_irand (context, ny-1) + 1;
						*x_to = 0;
						*y_to = iplace;
						break;
					case 1:
						iplace = p_my_irand (context, nx-1) + 1;
						*x_to = iplace;
						*y_to = ny+1;
						break;
					case 2:
						iplace = p_my_irand (context, ny-1) + 1;
						*x_to = nx+1;
						*y_to = iplace;
						break;
					case 3:
						iplace = p_my_irand (context, nx-1) + 1;
						*x_to = iplace;
						*y_to = 0;
						break;
					default:
						printf("Error in find_to.  Unexpected io swap location.\n");
						exit (1);
				}
			}
			else {   /* rlx is less than whole chip */
				if (x_from == 0) {
					iplace = p_my_irand (context, 2*rly);
					*y_to = y_from - rly + iplace;
					*x_to = x_from;
					if (*y_to > ny) {
						*y_to = ny + 1;
						*x_to = p_my_irand (context, rlx - 1) + 1;
					}
					else if (*y_to < 1) {
						*y_to = 0;
						*x_to = p_my_irand (context, rlx - 1) + 1;
					}
				}
				else if (x_from == nx+1) {
					iplace = p_my_irand (context, 2*rly);
					*y_to = y_from - rly + iplace;
					*x_to = x_from;
					if (*y_to > ny) {
						*y_to = ny + 1;
						*x_to = nx - p_my_irand (context, rlx - 1); 
					}
					else if (*y_to < 1) {
						*y_to = 0;
						*x_to = nx - p_my_irand (context, rlx - 1);
					}
				}
				else if (y_from == 0) {
					iplace = p_my_irand (context, 2*rlx);
					*x_to = x_from - rlx + iplace;
					*y_to = y_from;
					if (*x_to > nx) {
						*x_to = nx + 1;
						*y_to = p_my_irand (context, rly - 1) + 1;
					}
					else if (*x_to < 1) {
						*x_to = 0;
						*y_to = p_my_irand (context, rly -1) + 1;
					}
				}
				else {  /* *y_from == ny + 1 */
					iplace = p_my_irand (context, 2*rlx);
					*x_to = x_from - rlx + iplace;
					*y_to = y_from;
					if (*x_to > nx) {
						*x_to = nx + 1;
						*y_to = ny - p_my_irand (context, rly - 1);
					}
					else if (*x_to < 1) {
						*x_to = 0;
						*y_to = ny - p_my_irand (context, rly - 1);
					}
				}
			}    /* End rlx if */
		}    /* end type if */
	} while ((x_from == *x_to) && (y_from == *y_to));

#ifdef DEBUG
   if (*x_to < 0 || *x_to > nx+1 || *y_to < 0 || *y_to > ny+1) {
      printf("Error in routine find_to:  (x_to,y_to) = (%d,%d)\n",
            *x_to, *y_to);
      exit(1);
   }

   if (type == CLB) {
     if (clb[*x_to][*y_to].type != CLB) {
        printf("Error: Moving CLB to illegal type block at (%d,%d)\n",
          *x_to,*y_to);
        exit(1);
     }
   }
   else {
     if (clb[*x_to][*y_to].type != IO) {
        printf("Error: Moving IO block to illegal type location at "
              "(%d,%d)\n", *x_to, *y_to);
        exit(1);
     }
   }
#endif

/* printf("(%d,%d) moved to (%d,%d)\n",x_from,y_from,*x_to,*y_to); */
}

static int ptry_swap (struct pcontext *context) {

/* Picks some block and moves it to another spot.  If this spot is   *
 * occupied, switch the blocks.  Assess the change in cost function  *
 * and accept or reject the move.  If rejected, return 0.  If        *
 * accepted return 1.  Pass back the new value of the cost function. * 
 * rlim is the range limiter.  pins_on_block gives the number of     *
 * pins on each type of block (improves efficiency).                 */

	int b_from, x_to, y_to, x_from, y_from, b_to; 
	int off_from, k, inet, keep_switch, io_num, num_of_pins;
	int num_nets_affected, bb_index;
	float delta_c, bb_delta_c, timing_delta_c, delay_delta_c, newcost;

/* Allocate the local bb_coordinate storage, etc. only once. */

	if (context->bb_coord_new == NULL) {
		context->bb_coord_new = (struct s_bb *) my_malloc (2 * pins_per_clb * sizeof (struct s_bb));
		context->bb_edge_new = (struct s_bb *) my_malloc (2 * pins_per_clb * sizeof (struct s_bb));
		context->nets_to_update = (int *) my_malloc (2 * pins_per_clb * sizeof (int));
		context->net_block_moved = (int *) my_malloc (2 * pins_per_clb * sizeof (int));
	}


	b_from = p_my_irand(context, num_blocks - 1);

/* If the pins are fixed we never move them from their initial    *
 * random locations.  The code below could be made more efficient *
 * by using the fact that pins appear first in the block list,    *
 * but this shouldn't cause any significant slowdown and won't be *
 * broken if I ever change the parser so that the pins aren't     *
 * necessarily at the start of the block list.                    */

	if (context->fixed_pins == TRUE) {
		while (context->block[b_from].type != CLB) {
			b_from = p_my_irand(context, num_blocks - 1);
		}
	}

	x_from = context->block[b_from].x;
	y_from = block[b_from].y;
	find_to (context, x_from, y_from, context->block[b_from].type, context->rlim, &x_to, &y_to);

/* Make the switch in order to make computing the new bounding *
 * box simpler.  If the cost increase is too high, switch them *
 * back.  (block data structures switched, clbs not switched   *
 * until success of move is determined.)                       */

	if (context->block[b_from].type == CLB) {
		io_num = -1;            /* Don't need, but stops compiler warning. */
		if (context->clb[x_to][y_to].occ == 1) {    /* Occupied -- do a switch */
			b_to = context->clb[x_to][y_to].u.block;
			context->block[b_from].x = x_to;
			context->block[b_from].y = y_to;
			context->block[b_to].x = x_from;
			context->block[b_to].y = y_from; 
		}    
		else {
			b_to = EMPTY;
			context->block[b_from].x = x_to;
			context->block[b_from].y = y_to; 
		}
	}
	else {   /* io block was selected for moving */

		io_num = p_my_irand(context, io_rat - 1);
		if (io_num >= context->clb[x_to][y_to].occ) {  
			/* Moving to an empty location */
			b_to = EMPTY;
			context->block[b_from].x = x_to;
			context->block[b_from].y = y_to;
		}
		else {          /* Swapping two blocks */
			b_to = *(context->clb[x_to][y_to].u.io_blocks+io_num);
			context->block[b_to].x = x_from;
			context->block[b_to].y = y_from;
			context->block[b_from].x = x_to;
			context->block[b_from].y = y_to;
		}

	}

/* Now update the cost function.  May have to do major optimizations *
 * here later.                                                       */

/* I'm using negative values of temp_net_cost as a flag, so DO NOT   *
 * use cost functions that can go negative.                          */

	delta_c = 0;                    /* Change in cost due to this swap. */
	bb_delta_c = 0;
	timing_delta_c = 0;

	num_of_pins = context->pins_on_block[context->block[b_from].type];    

	num_nets_affected = find_affected_nets (context, b_from, b_to, num_of_pins);

	bb_index = 0;               /* Index of new bounding box. */

	for (k=0;k<num_nets_affected;k++) {
		inet = context->nets_to_update[k];

/* If we swapped two blocks connected to the same net, its bounding box *
 * doesn't change.                                                      */

	if (context->net_block_moved[k] == FROM_AND_TO) 
		continue;

	if (net[inet].num_pins <= SMALL_NET) {
		get_non_updateable_bb (context, inet, &(context->bb_coord_new[bb_index]));
	}
	else {
		if (context->net_block_moved[k] == FROM) 
			update_bb (context, inet, &(context->bb_coord_new[bb_index]), &(context->bb_edge_new[bb_index]), x_to, y_to, x_from, y_from);      
		else
			update_bb (context, inet, &(context->bb_coord_new[bb_index]), &(context->bb_edge_new[bb_index]), x_to, y_to, x_from, y_from);      
	}

	if (context->placer_opts->place_cost_type != NONLINEAR_CONG) {
		context->temp_net_cost[inet] = get_net_cost (inet, &(context->bb_coord_new[bb_index]));
		bb_delta_c += context->temp_net_cost[inet] - context->net_cost[inet];
	}

	bb_index++;
	}   

	delta_c = bb_delta_c;

 
	keep_switch = assess_swap (context, delta_c, context->t); 

 /* 1 -> move accepted, 0 -> rejected. */ 

	if (keep_switch) {
		context->cost = context->cost + delta_c;    
    context->bb_cost = context->bb_cost + bb_delta_c;


 /* update net cost functions and reset flags. */

		bb_index = 0;

		for (k=0;k<num_nets_affected;k++) {
			inet = context->nets_to_update[k];

/* If we swapped two blocks connected to the same net, its bounding box *
 * doesn't change.                                                      */

			if (context->net_block_moved[k] == FROM_AND_TO) {
				context->temp_net_cost[inet] = -1;  
				continue;
			}

			context->bb_coords[inet] = context->bb_coord_new[bb_index];
			if (net[inet].num_pins > SMALL_NET) 
				context->bb_num_on_edges[inet] = context->bb_edge_new[bb_index];

				bb_index++;

				context->net_cost[inet] = context->temp_net_cost[inet];
				context->temp_net_cost[inet] = -1;  
		}

 /* Update Clb data structures since we kept the move. */

		if (context->block[b_from].type == CLB) {
			if (b_to != EMPTY) {
				context->clb[x_from][y_from].u.block = b_to; 
				context->clb[x_to][y_to].u.block = b_from;
			}
			else {
				context->clb[x_to][y_to].u.block = b_from;   
				context->clb[x_to][y_to].occ = 1;
				context->clb[x_from][y_from].occ = 0; 
			}
		}

		else {     /* io block was selected for moving */

     /* Get the "sub_block" number of the b_from block. */

			for (off_from=0;;off_from++) {
				if (context->clb[x_from][y_from].u.io_blocks[off_from] == b_from) break;
			}

			if (b_to != EMPTY) {   /* Swapped two blocks. */
				context->clb[x_to][y_to].u.io_blocks[io_num] = b_from;
				context->clb[x_from][y_from].u.io_blocks[off_from] = b_to;
			}
			else {                 /* Moved to an empty location */
				context->clb[x_to][y_to].u.io_blocks[context->clb[x_to][y_to].occ] = b_from;  
				context->clb[x_to][y_to].occ++;   
				for  (k=off_from;k<context->clb[x_from][y_from].occ-1;k++) { /* prevent gap  */
					context->clb[x_from][y_from].u.io_blocks[k] =       /* in io_blocks */
						context->clb[x_from][y_from].u.io_blocks[k+1];
				}
				context->clb[x_from][y_from].occ--;
			}
		}  
	}  

	else {    /* Move was rejected.  */

/* Reset the net cost function flags first. */
		for (k=0;k<num_nets_affected;k++) {
			inet = context->nets_to_update[k];
			context->temp_net_cost[inet] = -1;
		}
    
 /* Restore the block data structures to their state before the move. */
		context->block[b_from].x = x_from;
		context->block[b_from].y = y_from;
		if (b_to != EMPTY) {
			context->block[b_to].x = x_to;
			context->block[b_to].y = y_to;
		}

/* Restore the region occupancies to their state before the move. */

	}
 
	return(keep_switch);
}

static float recompute_bb_cost (struct pcontext *context, int place_cost_type, int num_regions) {

/* Recomputes the cost to eliminate roundoff that may have accrued.  *
 * This routine does as little work as possible to compute this new  *
 * cost.                                                             */

 int i, j, inet;
 float cost;

 cost = 0;

/* Initialize occupancies to zero if regions are being used. */

 for (inet=0;inet<num_nets;inet++) {     /* for each net ... */
 
    if (is_global[inet] == FALSE) {    /* Do only if not global. */

       /* Bounding boxes don't have to be recomputed; they're correct. */ 
  
       if (place_cost_type != NONLINEAR_CONG) {
          cost += context->net_cost[inet];
       } 
    }
 }
 
 return (cost);
}


void *parallel_place (void *arg) {
	int inner_iter;
	struct pcontext *context = (struct pcontext *) arg;
	int iters = 0;
	int run_for = context->move_lim;
	
	if (context->update_freq < 1) {
		run_for *= context->update_freq;
	}
	
	while (exit_crit(context->t, context->cost, *(context->annealing_sched)) == 0) {
		context->av_cost = 0.;
		context->av_bb_cost = 0.;
		context->av_delay_cost = 0.;
		context->av_timing_cost = 0.;
		context->sum_of_squares = 0.;
		context->success_sum = 0;

		context->inner_crit_iter_count = 1;

		for (inner_iter=0; inner_iter < run_for; inner_iter++) {
			if (ptry_swap(context) == 1) {
				context->success_sum++;
				context->av_cost += context->cost;
				context->av_bb_cost += context->bb_cost;
				context->av_timing_cost += context->timing_cost;
				context->av_delay_cost += context->delay_cost;
				context->sum_of_squares += context->cost * context->cost;
			}
		}

/* Lines below prevent too much round-off error from accumulating *
 * in the cost over many iterations.  This round-off can lead to  *
 * error checks failing because the cost is different from what   *
 * you get when you recompute from scratch.                       */
 
		context->moves_since_cost_recompute += context->move_lim;
		if (context->moves_since_cost_recompute > MAX_MOVES_BEFORE_RECOMPUTE) {
			context->new_bb_cost = recompute_bb_cost (context, context->placer_opts->place_cost_type, context->placer_opts->num_regions);
			if (fabs(context->new_bb_cost - context->bb_cost) > context->bb_cost * ERROR_TOL) {
				printf("Error in try_place:  new_bb_cost = %g, old bb_cost = %g.\n",context->new_bb_cost, context->bb_cost);
				exit (1);
			}
			context->bb_cost = context->new_bb_cost;

			if (context->placer_opts->place_algorithm == BOUNDING_BOX_PLACE) {
				context->cost = context->new_bb_cost;
			}
			context->moves_since_cost_recompute = 0;
		}

		context->tot_iter += context->move_lim;
		context->success_rat = ((float) context->success_sum)/ context->move_lim;
		if (context->success_sum == 0) {
			context->av_cost = context->cost;
			context->av_bb_cost = context->bb_cost;
			context->av_timing_cost = context->timing_cost;
			context->av_delay_cost = context->delay_cost;
		}
		else {
			context->av_cost /= context->success_sum;
			context->av_bb_cost /= context->success_sum;
			context->av_timing_cost /= context->success_sum;
			context->av_delay_cost /= context->success_sum;
		}
		context->std_dev = get_std_dev (context->success_sum, context->sum_of_squares, context->av_cost);


#ifndef SPEC
    printf("%11.5g  %10.6g %11.6g  %11.6g  %11.6g %11.6g %11.4g %9.4g %8.3g  %7.4g  %7.4g  %10d  ",context->t, context->av_cost, 
	   context->av_bb_cost, context->av_timing_cost, context->av_delay_cost, context->place_delay_value, context->d_max, context->success_rat, context->std_dev, 
	   context->rlim, context->crit_exponent,context->tot_iter);
#endif


		context->oldt = context->t;  /* for finding and printing alpha. */

		update_t (&(context->t), context->std_dev, context->rlim, context->success_rat, *(context->annealing_sched));

#ifndef SPEC
    printf("%7.4g\n",context->t/context->oldt);
#endif

		sprintf(context->msg,"Cost: %g  BB Cost %g  TD Cost %g  Temperature: %g  d_max: %g",context->cost, context->bb_cost, context->timing_cost, context->t, context->d_max);
//		update_screen(MINOR, msg, PLACEMENT, FALSE);
		update_rlim (&(context->rlim), context->success_rat);

#ifdef VERBOSE 
 dump_clbs();
#endif

		iters++;
		if (context->update_freq < 1) {
			barrier_wait_run(&barrier, &merge_contexts, (void *)contexts);
		}
		else if (context->update_freq >= 1 && iters % (int)(context->update_freq) == 0) {
			barrier_wait_run(&barrier, &merge_contexts, (void *)contexts);
		}


	}
	barrier_wait_run(&barrier, &merge_contexts, (void *)contexts);
}

void merge_contexts (void *ptr) {
	struct pcontext *arr = (struct pcontext *)ptr;
	int i, best_cost, best_index;
	
	best_cost = arr[0].cost;
	best_index = 0;
	
	for (i=1; i<num_threads; i++) {
		if (arr[i].cost < best_cost) {
			best_cost = arr[i].cost;
			best_index = i;
		}
	}
	
	copy_context(arr, &(arr[best_index]), num_threads);
}

void copy_context (struct pcontext *arr, struct pcontext *context, int n) {
	int cont, i, j, k;
	
 for (cont=0; cont<n; cont++) {
		for (i=0; i<num_blocks; i++) {
			arr[cont].block[i].type = context->block[i].type;
			arr[cont].block[i].x = context->block[i].x;
			arr[cont].block[i].y = context->block[i].y;
		}
	
	
	for (i=0; i<=nx+1; i++) {
		for (j=0; i<=ny+1; j++) {
			arr[cont].clb[i][j].type = context->clb[i][j].type;
			arr[cont].clb[i][j].occ = context->clb[i][j].occ;
			arr[cont].clb[i][j].u.block = context->clb[i][j].u.block;
			

			for (k=0; k<clb[i][j].occ; k++) {
				arr[cont].clb[i][j].u.io_blocks[k] = context->clb[i][j].u.io_blocks[k];
			}
		}
	}
	
	for (i=0; i<num_nets; i++) {
		arr[cont].bb_coords[i].xmin = context->bb_coords[i].xmin;
		arr[cont].bb_coords[i].xmax = context->bb_coords[i].xmax;	
		arr[cont].bb_coords[i].ymin = context->bb_coords[i].ymin;
		arr[cont].bb_coords[i].ymax = context->bb_coords[i].ymax;	
		arr[cont].net_cost[i] = context->net_cost[i];
		arr[cont].temp_net_cost[i] = context->temp_net_cost[i];
	}

	arr[cont].tot_iter = context->tot_iter;
	arr[cont].success_sum = context->success_sum;
	arr[cont].move_lim = context->move_lim;
	arr[cont].moves_since_cost_recompute = context->moves_since_cost_recompute;
	arr[cont].t = context->t;
	arr[cont].success_rat = context->success_rat;
	arr[cont].rlim = context->rlim;
	arr[cont].cost = context->cost;
	arr[cont].timing_cost = context->timing_cost;
	arr[cont].delay_cost = context->delay_cost;
	arr[cont].bb_cost = context->bb_cost;
	arr[cont].new_bb_cost = context->new_bb_cost;
	arr[cont].new_timing_cost = context->new_timing_cost;
	
	arr[cont].delay_cost = context->delay_cost;
	arr[cont].new_delay_cost = context->new_delay_cost;
	arr[cont].place_delay_value = context->place_delay_value;
	arr[cont].oldt = context->oldt;

	arr[cont].av_cost = context->av_cost;
	arr[cont].av_bb_cost = context->av_bb_cost;
	arr[cont].av_timing_cost = context->av_timing_cost;
	arr[cont].av_delay_cost = context->av_delay_cost;
	arr[cont].sum_of_squares = context->sum_of_squares;
	arr[cont].std_dev = context->std_dev;

	arr[cont].fixed_pins = context->fixed_pins;
	
	arr[cont].inet = context->inet;
	arr[cont].outer_crit_iter_count = context->outer_crit_iter_count;
	arr[cont].inner_crit_iter_count = context->inner_crit_iter_count;
	arr[cont].inner_recompute_limit = context->inner_recompute_limit;
	
 }
}

void alloc_context (struct pcontext *context, float update_freq,
	float inverse_prev_bb_cost, float inverse_prev_timing_cost,
	struct s_annealing_sched *annealing_sched, struct s_placer_opts *placer_opts,
	float *net_cost, float *temp_net_cost, float cost, float bb_cost,
	float timing_cost, float delay_cost, float rlim, int *duplicate_pins, int **unique_pin_list, float d_max, float place_delay_value, float crit_exponent, int *pins_on_block, struct s_bb *bb_coords, struct s_bb *bb_num_on_edges)
{
	int i, j, k, *index;
	int num_pins;
	
	/*
	context->net = (struct s_net *) my_malloc (num_nets*sizeof(struct s_net));
	assert(context->net);
	for (i=0; i<num_nets; i++) {
		context->net[i].name = (char *) my_malloc (strlen(net[i].name)*sizeof(char));
		assert(context->net[i].name);
		strcpy(context->net[i].name, net[i].name);
		num_pins = net[i].num_pins;
		context->net[i].num_pins = num_pins;
		context->net[i].blocks = (int *) my_malloc (num_pins*sizeof(int));
		assert(context->net[i].blocks);
		context->net[i].blk_pin = (int *) my_malloc (num_pins*sizeof(int));
		assert(context->net[i].blk_pin);
		
		for (j=0; j<num_pins; j++) {
			context->net[i].blocks[j] = net[i].blocks[j];
			context->net[i].blk_pin[j] = net[i].blocks[j];
		}
	}
	*/
	context->crit_exponent = crit_exponent;
	context->place_delay_value = place_delay_value;
	context->d_max = d_max;
	context->unique_pin_list = unique_pin_list;
	context->duplicate_pins = duplicate_pins;
	context->rlim = rlim;
	context->cost = cost;
	context->bb_cost = bb_cost;
	context->timing_cost = timing_cost;
	context->delay_cost = delay_cost;	
	fill_rand_buf(context->rand_buf, &(context->first_rand));
	context->nets_to_update = NULL;
	context->net_block_moved = NULL;

	context->bb_coord_new = NULL;
	context->bb_edge_new = NULL;
	context->update_freq = update_freq;
	context->inverse_prev_bb_cost = inverse_prev_bb_cost;
	context->inverse_prev_timing_cost = inverse_prev_timing_cost;

	context->annealing_sched = annealing_sched;
	context->placer_opts = placer_opts;
	
	context->block = (struct s_block *) my_malloc (num_blocks*sizeof(struct s_block));
	if (context->block == NULL) exit(1);
	
	for (i=0; i<3; i++) {
		context->pins_on_block[i] = pins_on_block[i];
	}
	
	for (i=0; i<num_blocks; i++) {
		context->block[i].name = (char *) my_malloc (strlen(block[i].name)*sizeof(char));
		if (context->block[i].name == NULL) exit(1);
		strcpy(context->block[i].name, block[i].name);
		context->block[i].type = block[i].type;
		context->block[i].x = block[i].x;
		context->block[i].y = block[i].y;
		context->block[i].nets = (int *) my_malloc (pins_per_clb * sizeof(int));
		if (context->block[i].nets == NULL) exit(1);
		for (j=0; j<pins_per_clb; j++) {
			context->block[i].nets[j] = block[i].nets[j];
		}
	}
	
	context->clb = (struct s_clb **) alloc_matrix (0, nx+1, 0, ny+1, sizeof(struct s_clb));
			
	i = 2*io_rat*(nx+ny);
	index = (int *) my_malloc (i*sizeof(int));
	for (i=1;i<=nx;i++) {
		context->clb[i][0].u.io_blocks = index;
		index+=io_rat;
		context->clb[i][ny+1].u.io_blocks = index;
		index+=io_rat;
	}
	for (i=1;i<=ny;i++) {
		context->clb[0][i].u.io_blocks = index;
		index+=io_rat;
		context->clb[nx+1][i].u.io_blocks = index;
		index+=io_rat;
	}
	
	for (i=0; i<=nx+1; i++) {
		for (j=0; i<=ny+1; j++) {
			context->clb[i][j].type = clb[i][j].type;
			context->clb[i][j].occ = clb[i][j].occ;
			context->clb[i][j].u.block = clb[i][j].u.block;
			if (clb[i][j].type == IO) {
				for (k=0; k<clb[i][j].occ; k++) {
					context->clb[i][j].u.io_blocks[k] = clb[i][j].u.io_blocks[k];
				}
			}
		}
	}

	context->bb_coords = (struct s_bb *) my_malloc (num_nets * sizeof(struct s_bb));
	context->bb_num_on_edges = (struct s_bb *) my_malloc (num_nets * sizeof(struct s_bb));
	
	for (i=0; i<num_nets; i++) {
		context->bb_coords[i].xmin = bb_coords[i].xmin;
		context->bb_coords[i].xmax = bb_coords[i].xmax;	
		context->bb_coords[i].ymin = bb_coords[i].ymin;
		context->bb_coords[i].ymax = bb_coords[i].ymax;	
	}
	
	context->net_cost = (float *) my_malloc (num_nets * sizeof (float));
	context->temp_net_cost = (float *) my_malloc (num_nets * sizeof (float));
	
	for (i=0; i<num_nets; i++) {
		context->net_cost[i] = net_cost[i];
		context->temp_net_cost[i] = temp_net_cost[i];
	}

}

void restore_context (struct pcontext *context, float *cost, float *bb_cost, float *timing_cost, float *delay_cost, float *rlim, int *pins_on_block, float *net_cost, float *temp_net_cost, struct s_bb *bb_coords, struct s_bb *bb_num_on_edges)
{
	int i, j,k, *index;
	int num_pins;
	
	/*
	context->net = (struct s_net *) my_malloc (num_nets*sizeof(struct s_net));
	assert(context->net);

	for (i=0; i<num_nets; i++) {
		context->net[i].name = (char *) my_malloc (strlen(net[i].name)*sizeof(char));
		assert(context->net[i].name);
		strcpy(context->net[i].name, net[i].name);
		num_pins = net[i].num_pins;
		context->net[i].num_pins = num_pins;
		context->net[i].blocks = (int *) my_malloc (num_pins*sizeof(int));
		assert(context->net[i].blocks);
		context->net[i].blk_pin = (int *) my_malloc (num_pins*sizeof(int));
		assert(context->net[i].blk_pin);
		
		for (j=0; j<num_pins; j++) {
			context->net[i].blocks[j] = net[i].blocks[j];
			context->net[i].blk_pin[j] = net[i].blocks[j];
		}
	}
	*/	
	
	*cost = context->cost;
	*bb_cost = context->bb_cost;
	*timing_cost = context->timing_cost;
	*delay_cost = context->delay_cost;
	*rlim = context->rlim;
	
	for (i=0; i<num_blocks; i++) {
		block[i].x = context->block[i].x;
		block[i].y = context->block[i].y;
		for (j=0; j<pins_per_clb; j++) {
			block[i].nets[j] = context->block[i].nets[j];
		}
	}
	
	for (i=0; i<=nx+1; i++) {
		for (j=0; i<=ny+1; j++) {
			clb[i][j].occ = context->clb[i][j].occ;
			clb[i][j].u.block = context->clb[i][j].u.block;
			clb[i][j].type = context->clb[i][j].type;
			
			if (clb[i][j].type == IO) {
				for (k=0; k<clb[i][j].occ; k++) {
					clb[i][j].u.io_blocks[k] = context->clb[i][j].u.io_blocks[k];
				}
			}
		}
	}
	
	for (i=0; i<num_nets; i++) {
		bb_coords[i].xmin = context->bb_coords[i].xmin;
		bb_coords[i].xmax = context->bb_coords[i].xmax;	
		bb_coords[i].ymin = context->bb_coords[i].ymin;
		bb_coords[i].ymax = context->bb_coords[i].ymax;	
		net_cost[i] = context->net_cost[i];
		temp_net_cost[i] = context->temp_net_cost[i];
	}

}

void free_context (struct pcontext *context)
{
	int i, j, *index;
	int num_pins;
	
	/*
	context->net = (struct s_net *) my_malloc (num_nets*sizeof(struct s_net));
	assert(context->net);

	for (i=0; i<num_nets; i++) {
		context->net[i].name = (char *) my_malloc (strlen(net[i].name)*sizeof(char));
		assert(context->net[i].name);
		strcpy(context->net[i].name, net[i].name);
		num_pins = net[i].num_pins;
		context->net[i].num_pins = num_pins;
		context->net[i].blocks = (int *) my_malloc (num_pins*sizeof(int));
		assert(context->net[i].blocks);
		context->net[i].blk_pin = (int *) my_malloc (num_pins*sizeof(int));
		assert(context->net[i].blk_pin);
		
		for (j=0; j<num_pins; j++) {
			context->net[i].blocks[j] = net[i].blocks[j];
			context->net[i].blk_pin[j] = net[i].blocks[j];
		}
	}
	*/
	
	free(context->pins_on_block);
	
	for (i=0; i<num_blocks; i++) {
		free(context->block[i].name);
		free(context->block[i].nets);
	}
	free(context->block);

	index = context->clb[i][0].u.io_blocks;
	free(index);
	
	alloc_matrix (0, nx+1, 0, ny+1, sizeof(struct s_clb));
	free_matrix(context->clb, 0, nx+1, 0, sizeof(struct s_clb));

	free(context->bb_coords);
	free(context->bb_num_on_edges);
		
	free(context->net_cost);
	free(context->temp_net_cost);

}
