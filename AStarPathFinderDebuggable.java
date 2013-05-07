package com.qylla.util.pathfinding;

import java.util.ArrayList;
import java.util.Collections;

import org.anddev.andengine.entity.layer.tiled.tmx.TMXLayer;
import org.anddev.andengine.entity.layer.tiled.tmx.TMXProperties;
import org.anddev.andengine.entity.layer.tiled.tmx.TMXTile;
import org.anddev.andengine.entity.layer.tiled.tmx.TMXTileProperty;
import org.anddev.andengine.entity.layer.tiled.tmx.TMXTiledMap;
import org.anddev.andengine.entity.primitive.Rectangle;
import org.anddev.andengine.entity.scene.Scene;
import org.anddev.andengine.entity.text.Text;
import org.anddev.andengine.opengl.font.Font;
import org.anddev.andengine.util.HorizontalAlign;

import com.qylla.games.androidwars.entities.TMXTileMapObject;
import com.qylla.games.androidwars.testclasses.Unit;
import com.qylla.util.pathfinding.AStarPathFinder.Node;
import com.qylla.util.pathfinding.interfaces.Mover;
import com.qylla.util.pathfinding.interfaces.PathFinder;

/**
 * A path finder implementation that uses the AStar heuristic based algorithm
 * to determine a path. 
 * 
 * @author Cesar Ramirez
 */
public class AStarPathFinderDebuggable extends AStarPathFinder{
	public Scene scene;
	public Font font;

	/**
	 * Create a path finder 
	 * 
	 * @param map The map to be searched
	 * @param allowDiagMovement True if the search should try diaganol movement
	 */
	public AStarPathFinderDebuggable(TMXTileMapObject map, boolean allowDiagMovement) {
		super(map, allowDiagMovement);
	}
	
	/**
	 * @see PathFinder#findPath(Mover, int, int, int, int)
	 */
	public Path findPath(Mover mover, int maxDistance, int sx, int sy, int tx, int ty) {	
		// easy first check, if the destination is blocked, we can't get there
		if (!this.isValidLocation(mover, tx, ty)) {
			return null;
		}else{
			// If the distance of a straight line draw from the origin to the destination tile 
			// is longer than the maxDistance to move, we can't get there.
			if (Math.abs(tx - sx) > maxDistance || Math.abs(ty - sy) > maxDistance || Math.abs(tx - sx) + Math.abs(ty - sy) > maxDistance) {
				return null;
			}
		}
		 
		// The initial state for A*. The closed group is empty. Only the starting
		// tile is in the open list.
		closed.clear();
		open.clear();
		nodes[sx][sy].depth = 0;
		open.add(nodes[sx][sy]);
		
		//We haven't found any tile that reaches the destiny yet
		nodes[tx][ty].parent = null;
		
		//This loop will iterate until we run out of tiles that can
		//be evaluated.
		while ((open.size() != 0)) {
			Node current = getFirstInOpen();
			
			if (current == nodes[tx][ty]) {
				break;
			}
			
			removeFromOpen(current);
			addToClosed(current);
			
			//We get the distance from the current tile to the destination tile
			int currentDistanceX = (tx - current.x);
			int currentDistanceY = (ty - current.y);
				
			//This array will store the coordinates of the next movements
			//depending of the X or Y distance to the destination
			int[] order = new int[4];
			if(Math.abs(currentDistanceX) > Math.abs(currentDistanceY)){
				if(currentDistanceX < 0){
					order[0] = 3;	
					order[1] = 2;	
					order[2] = 4;	
					order[3] = 1;	
				}else{
					order[0] = 1;	
					order[1] = 3;	
					order[2] = 2;	
					order[3] = 4;
				}
			}else{
				if(currentDistanceY < 0){
					order[0] = 4;	
					order[1] = 1;	
					order[2] = 3;	
					order[3] = 2;
				}else{
					order[0] = 2;	
					order[1] = 1;	
					order[2] = 3;	
					order[3] = 4;
				}
			}
						
			int x = 0;
			int y = 0;
			
			// search through all the neighbors of the current node evaluating
			// them as next steps
			for (int i = 0;i < 4; i++) {
				switch(order[i]){
				case 1:
					x = 1;
					y = 0;
					break;
				case 2:
					x = 0;
					y = 1;
					break;
				case 3:
					x = -1;
					y = 0;
					break;
				case 4:
					y = -1;
					x = 0;
					break;
				}
					// not a neighbor, its the current tile
					if ((x == 0) && (y == 0)) {
						continue;
					}
					
					// if we're not allowing diagonal movement then only 
					// one of x or y can be set
					if (!allowDiagMovement) {
						if ((x != 0) && (y != 0)) {
							continue;
						}
					}
					
					// determine the location of the neighbor and evaluate it
					int Nx = x + current.x;
					int Ny = y + current.y;
					

					// The neighbor tile will only evaluated if is a valid place and if is not outside of the movement range
					// This last check will be done again.
					if (isValidLocation(mover,Nx,Ny) && (Math.abs(tx - Nx) + Math.abs(ty - Ny) <= maxDistance)) {						
						// the cost to get to this node is cost the cost to reach this node. 
						Node neighbour = nodes[Nx][Ny];
						int nextStepCost = (int) (neighbour.cost + current.depth);
						
						// If the current evaluation finds that this movement have less cost than
						// previously evaluated, then the tile can be evaluated again to find a better path.
						if (nextStepCost < neighbour.depth) {
							if (inOpenList(neighbour)) {
								removeFromOpen(neighbour);
							}
							if (inClosedList(neighbour)) {
								removeFromClosed(neighbour);
							}
						}
						
						if (!inOpenList(neighbour) && !(inClosedList(neighbour))) {
							neighbour.heuristic = (float) Math.sqrt((currentDistanceX * currentDistanceX)+(currentDistanceY*currentDistanceY));
							// If moving to the neighbor tile doenst exceeds our movement range, then can be added.
							// This check is done twice in case this tile has been reevaluated previously for a better path.
							if(neighbour.cost + current.depth <= maxDistance)
							{
								neighbour.setParent(current);
								addToOpen(neighbour);
								
								try {
									Thread.sleep(100);
									final Rectangle currentTileRectangle = new Rectangle(Nx*52, Ny*52, 52, 52);
									currentTileRectangle.setColor(0, 1, 0, 0.25f);
									final Text textCenter = new Text(Nx*52+1, Ny*52+1, font, new Integer(nodes[Nx][Ny].depth).toString(), HorizontalAlign.CENTER);
									final Text textCenter2 = new Text(Nx*52+1, Ny*52+18, font, new Integer((int) nodes[Nx][Ny].cost).toString(), HorizontalAlign.CENTER);
									scene.getTopLayer().addEntity(currentTileRectangle);
									scene.getTopLayer().addEntity(textCenter);
									scene.getTopLayer().addEntity(textCenter2);
								} catch (InterruptedException e) {
									// TODO Auto-generated catch block
								}
							}
						}
					}
			}
		}

		// since we've got an empty open list or we've run out of search 
		// there was no path. Just return null
		if (nodes[tx][ty].parent == null) {
			return null;
		}
		
		// At this point we've definitely found a path so we can uses the parent
		// references of the nodes to find out way from the target location back
		// to the start recording the nodes on the way.
		Path path = new Path();
		Node target = nodes[tx][ty];
		while (target != nodes[sx][sy]) {
			path.prependStep(target.x, target.y);
			target = target.parent;
		}
		path.prependStep(sx,sy);
		
		// thats it, we have our path 
		return path;
	}

	public void setScene(Scene scene){
		this.scene = scene;
	}
	
	public void serFont(Font font){
		this.font = font;
	}
}