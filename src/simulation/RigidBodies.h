#pragma once
#include "SimulationConfig.h"
#include <box2d/id.h>
#include <box2d/math_functions.h>
#include <array>
#include <bitset>
#include <cstdint>
#include <unordered_map>
#include <vector>

class Simulation;
struct b2Manifold;

// Rigid bodies made of rigid material pixels (RIGD, RWOD, RICE...), simulated
// with Box2D.
//
// Every frame, connected pixels of the same material are grouped into bodies
// (see Gather), solids and walls around each body are mirrored into Box2D as
// static shapes, gravity, buoyancy and drag are applied, Box2D steps, and each
// body's pixels are placed back on the grid at its new pose (see Place).
// Liquids and gases aren't obstacles: bodies move into them, and the liquid or
// gas they push out of the way goes into the cells the bodies leave.
//
// Each pixel stores its body ID in tmp2 and its body-local coordinates in tmp3
// and tmp4. Box2D state is not saved: after loading a save, bodies are rebuilt
// from these fields, at rest but with their shapes and identities intact.
class RigidBodies
{
public:
	RigidBodies();
	~RigidBodies();
	RigidBodies(const RigidBodies &) = delete;
	RigidBodies &operator =(const RigidBodies &) = delete;

	void Update(Simulation &sim);
	void Clear();

	int BodyCount() const
	{
		return int(bodies.size());
	}

	// Seconds spent in each phase of Update, summed over all frames.
	struct Timings
	{
		double gather = 0, terrain = 0, step = 0, place = 0;
		int fluidLost = 0; // liquid or gas particles that had nowhere to go and were removed
		long fluidMoved = 0; // liquid or gas particles pushed aside by bodies
		int stuck = 0;  // body moves given up on because the pixels didn't fit
		int pulled = 0; // times Box2D was pulled back to where the pixels are
		long placed = 0, pending = 0, searchSteps = 0;
	};

	int DisplacedPixels() const
	{
		int n = 0;
		for (auto &[id, body] : bodies)
		{
			n += body.displaced;
		}
		return n;
	}
	Timings timings;

	// Move liquid from one side of a floating body to the other when it
	// stands higher on one side (see LevelLiquids). Simulation Options.
	bool levelLiquids = true;

	// Slow bodies moving through still air (see ApplyAirForces); wind and
	// pressure push them either way. Simulation Options.
	bool airResistance = true;

	// For tests: a body's Box2D position and velocity, by body ID.
	std::pair<b2Vec2, b2Vec2> PoseOf(int id) const;

	// For the PUSH tool: shove every body with a pixel in the given cells,
	// at `at`. Push shoves each along `speed`; Poke shoves each `speed`
	// straight away from `at`. A body as dense as water is sent off at that
	// speed (pixels per frame), whatever its size; denser ones go slower.
	void Push(const Simulation &sim, const std::vector<std::pair<int, int>> &cells, b2Vec2 at, b2Vec2 speed);
	void Poke(const Simulation &sim, const std::vector<std::pair<int, int>> &cells, b2Vec2 at, float speed);

	// For the GRAB tool: take hold of the body with a pixel in the given
	// cells nearest `at`, at that pixel; pull it towards wherever the hold is
	// moved, as if by a stiff spring (it still collides with things on the
	// way); let go of it, moving as it was, so it can be thrown. A hold that
	// isn't moved for a few frames lets go by itself.
	bool Grab(const Simulation &sim, const std::vector<std::pair<int, int>> &cells, b2Vec2 at);
	void MoveGrab(b2Vec2 to);
	void Release();

private:
	struct Pixel
	{
		int id;     // particle ID
		int lx, ly; // body-local coordinates
		int wx, wy; // cell the pixel was last placed in
		bool alive;
	};

	struct Body
	{
		int id;
		int material; // element type of its pixels
		b2BodyId b2;
		std::vector<Pixel> pixels;
		int alive;       // pixels found intact this frame
		int displaced;   // pixels moved aside from where the pose puts them
		std::vector<bool> mask; // which body-local cells the body has, from (maskX, maskY)
		int maskX, maskY, maskW, maskH;
		float reach;     // furthest pixel from the body's origin
		b2Vec2 gravity;  // gravity field it was last under, to notice changes
		b2Vec2 drawnPos; // Box2D pose the pixels were last placed at
		b2Rot drawnRot;
	};

	struct Loose
	{
		int id;
		int x, y;
		float vx, vy; // px/s
		int tag;      // 0 for brand new pixels
		int material;
		bool fresh;   // something other than rigid material last frame
	};

	static constexpr int tileSize = 16;
	static constexpr int tilesX = (XRES + tileSize - 1) / tileSize;
	static constexpr int tilesY = (YRES + tileSize - 1) / tileSize;

	struct Tile
	{
		std::bitset<tileSize * tileSize> solid;
		std::vector<b2ShapeId> shapes;
		bool built = false;
	};

	b2WorldId world;
	b2BodyId terrain;
	std::array<Tile, tilesX * tilesY> tiles;
	std::unordered_map<int, Body> bodies;
	std::vector<int> grains; // particle IDs of lone rigid pixels, rebuilt by Gather
	// What each particle slot held last frame, to tell when something (molten
	// gold, say) has just turned into rigid material.
	enum : uint8_t { noParticle, otherParticle, rigidParticle };
	std::vector<uint8_t> lastState;
	int lastActive = 0;
	std::vector<int> partBody;  // particle ID -> body ID, 0 if none
	std::vector<int> partIndex; // particle ID -> index in its body's pixels
	std::vector<uint32_t> claimStamp; // cells claimed by a body this pass, see Place; also scratch for Gather
	std::vector<int> claimBody;
	uint32_t stamp = 0;
	std::vector<uint32_t> visited; // cells searched when moving a pixel aside, see Place
	uint32_t visitStamp = 0;
	std::vector<uint32_t> sideStamp; // which side of a body each cell is on, see PlaceStep
	std::vector<int> side;
	uint32_t sidePass = 0;
	int grabbed = 0; // body ID held by the GRAB tool, 0 if none
	b2JointId grabJoint = b2_nullJointId;
	int grabFrame = 0; // last frame the hold was moved
	int nextBodyId = 1;
	int frame = 0;

	void Gather(Simulation &sim);
	void MoveGrains(Simulation &sim);
	void Dissolve(Body &body, std::vector<Loose> &into, int tag);
	void Bake(Simulation &sim, std::vector<Loose> &pixels);
	void RefreshTerrain(Simulation &sim);
	bool TerrainSolid(const Simulation &sim, int x, int y) const;
	void ApplyGravity(Simulation &sim);
	void ApplyFluidForces(Simulation &sim);
	void ApplyAirForces(Simulation &sim);
	static bool TerrainPreSolve(b2ShapeId shapeA, b2ShapeId shapeB, b2Manifold *manifold, void *context);
	bool KeepTerrainContact(b2ShapeId shapeA, b2ShapeId shapeB, b2Manifold &manifold) const;
	const Simulation *stepSim = nullptr; // during b2World_Step, for TerrainPreSolve
	std::vector<int> BodiesIn(const Simulation &sim, const std::vector<std::pair<int, int>> &cells) const;
	void LevelLiquids(Simulation &sim);
	struct PlaceTarget
	{
		Body *body;
		b2Vec2 pos;
		b2Rot rot;
		bool *stuck; // set if the body couldn't be placed
		bool *byBodies; // and if only other rigid bodies were in its way
	};
	void Place(Simulation &sim);
	void PlaceStep(Simulation &sim, const std::vector<PlaceTarget> &targets);
	bool FluidNear(const Simulation &sim, const Body &body, b2Vec2 pos, float moved) const;
	void Place(const Body &body, b2Vec2 pos, b2Rot rot, std::vector<int> &cells);
	std::vector<std::pair<float, int>> placeScores; // scratch for Place
	std::vector<char> placeState;
	std::vector<int> placeSeen;
};
