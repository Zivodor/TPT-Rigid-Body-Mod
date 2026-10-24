#include "RigidBodies.h"
#include "Simulation.h"
#include "ElementClasses.h"
#include "SimulationData.h"
#include <box2d/box2d.h>
#include <algorithm>
#include <climits>
#include <optional>
#include <chrono>
#include <cmath>
#include <numbers>
#include <numeric>

namespace
{
	constexpr float frameRate = 60.0f; // Box2D time is TPT frames / frameRate
	constexpr float timeStep = 1.0f / frameRate;
	constexpr int subSteps = 8;
	// Acceleration under normal gravity, per frame squared. Comparable to heavy powders.
	constexpr float gravityPerFrame = 0.2f;
	constexpr float gravityScale = gravityPerFrame * frameRate * frameRate;
	constexpr float maxSpeed = 10.0f * frameRate;
	constexpr float terrainFriction = 0.6f;
	// Fewer pixels than this aren't a rigid body but loose grains.
	constexpr int minBodyPixels = 2;
	// How quickly liquid slows a body moving through it, per second, for a
	// body as dense as the liquid. Lighter bodies are slowed more.
	constexpr float fluidDrag = 10.0f;
	// Air: each face of a body open to the air is dragged towards the air's
	// velocity (scaled up as much as powders follow it), and every face is
	// pushed in by the pressure beside it. Being forces on the surface, they
	// barely move dense, chunky bodies (gold, stone) and blow light, thin
	// ones (foam) about. Per face, in TPT units (mass is pixels times
	// density, time is frames).
	constexpr float airDrag = 0.05f;
	constexpr float airSpeed = 6.0f;
	constexpr float airPressure = 0.1f;
	// TPT's air is never quite still; less than this counts as calm, so a
	// body resting or floating in calm air isn't nudged about by the noise.
	constexpr float calmAirSpeed = 0.05f;
	constexpr float calmPressure = 0.05f;
	// How far out from a body's side buoyancy looks for the liquid level.
	constexpr int fluidProbe = 4;
	// How far out from a floating body's ends liquid levels are compared, and
	// the most liquid particles moved from one side to the other per frame.
	constexpr int levelReach = 8;
	constexpr int maxLevelMoves = 4;
	// Terrain is mirrored into Box2D this far around each body, plus however
	// far the body could travel in a couple of frames.
	constexpr int terrainMargin = 4;
	constexpr int sleepingTerrainInterval = 8;
	// A body whose pixels could not be placed is pulled back to where they
	// are once Box2D has taken it further than this from them.
	constexpr float maxDrift = 2.0f;
	// Pixels that may be moved aside when placing a body, and how many
	// cells may be searched to find each one a free spot.
	constexpr int maxDisplaced = 3;
	constexpr size_t maxSearch = 4096;
	// Cells searched for a free spot near where a pixel lands when placing a body.
	constexpr size_t maxNearbySearch = 48;
	// Cells searched past the first open one for a lower one, when finding
	// where pushed-aside liquid goes.
	constexpr size_t levelSearch = 4000;
	// Most cells searched to find places for the liquid one body pushes aside.
	constexpr size_t maxFluidSearch = 1 << 17;
	// How deep into a body to look for the side liquid it moved over came
	// from.
	constexpr int maxSideSearch = 8;
	// A body's pixels are only moved once it has moved this far from where
	// they were drawn. Without this, a body resting near a half-pixel boundary
	// flickers between two rows, shoving liquid around every frame.
	constexpr float placeThreshold = 0.6f;
	// Bodies turned less than this far (in pixels at their furthest point)
	// from square are drawn square.
	constexpr float squareSnap = 1.0f;
	// Furthest any pixel moves in one step when a body sweeps through liquid,
	// and the most steps in a frame.
	constexpr float maxPlaceStep = 1.0f;
	constexpr int maxPlaceSteps = 16;

	struct Material
	{
		int type;
		float density; // relative to water
		float friction;
		float restitution;
	};

	// Densities are roughly the real ones, so things float and sink where
	// you'd expect: iron floats on mercury, ice floats on water but not oil.
	constexpr Material materials[] = {
		{ PT_RIGD,  7.8f, 0.6f,  0.1f  }, // iron
		{ PT_RWOD,  0.6f, 0.7f,  0.15f }, // wood
		{ PT_RRBR,  1.1f, 1.2f,  0.8f  }, // rubber
		{ PT_RICE, 0.92f, 0.02f, 0.05f }, // ice
		{ PT_RSTN,  2.6f, 0.9f,  0.05f }, // stone
		{ PT_RFOM,  0.1f, 0.7f,  0.3f  }, // foam
		{ PT_RGLD, 19.3f, 0.5f,  0.05f }, // gold
	};

	const Material *MaterialOf(int type)
	{
		for (auto &material : materials)
		{
			if (material.type == type)
			{
				return &material;
			}
		}
		return nullptr;
	}

	// The material of a rigid pixel (including a sparked one), or 0.
	// Lone pixels, moving like powder rather than as bodies, have this tag.
	constexpr int grainTag = INT_MIN;

	int RigidMaterial(const Particle &part)
	{
		if (MaterialOf(part.type))
		{
			return part.type;
		}
		if (part.type == PT_SPRK && MaterialOf(part.ctype))
		{
			return part.ctype;
		}
		return 0;
	}

	// Liquids and gases don't stop bodies; bodies push them aside.
	bool IsFluid(int type)
	{
		return SimulationData::CRef().elements[type].Properties & (TYPE_LIQUID | TYPE_GAS);
	}

	// Density relative to water, for buoyancy. Roughly the real values for
	// the liquids that have one; gases are too light to matter.
	float FluidDensity(int type)
	{
		if (!(SimulationData::CRef().elements[type].Properties & TYPE_LIQUID))
		{
			return 0.0f;
		}
		switch (type)
		{
		case PT_MERC: return 13.5f;
		case PT_LAVA: return 3.1f;
		case PT_EXOT: return 2.0f;
		case PT_NITR: return 1.6f;
		case PT_PSTE: return 1.6f;
		case PT_LRBD: return 1.46f;
		case PT_ACID: return 1.2f;
		case PT_LO2:  return 1.14f;
		case PT_DEUT: return 1.1f;
		case PT_GEL:  return 1.1f;
		case PT_SLTW: return 1.03f;
		case PT_MWAX: return 0.9f;
		case PT_OIL:  return 0.85f;
		case PT_DESL: return 0.83f;
		case PT_LNTG: return 0.81f;
		}
		return 1.0f;
	}

	int Round(float v)
	{
		return int(std::floor(v + 0.5f));
	}

	// How far a smoothed edge may stray from the pixel staircase it replaces.
	constexpr float edgeTolerance = 1.0f;
	// Corner rounding on every shape. Shapes that meet edge to edge (terrain
	// tiles, bands of a body) otherwise catch on each other's corners as one
	// slides across the other: Box2D's "ghost collisions".
	constexpr float cornerRadius = 0.4f;

	// The polygon with the same outline but corners rounded by cornerRadius,
	// or as it is if it's too thin for that.
	b2Polygon Rounded(const b2Polygon &sharp)
	{
		b2Vec2 inset[B2_MAX_POLYGON_VERTICES] = {};
		auto n = sharp.count;
		for (int i = 0; i < n; i++)
		{
			auto a = sharp.normals[(i + n - 1) % n], b = sharp.normals[i];
			auto d = 1.0f + b2Dot(a, b);
			if (d < 0.1f)
			{
				return sharp;
			}
			inset[i] = b2MulSub(sharp.vertices[i], cornerRadius / d, b2Add(a, b));
		}
		auto hull = b2ComputeHull(inset, n);
		if (hull.count != n)
		{
			return sharp;
		}
		auto rounded = b2MakePolygon(&hull, cornerRadius);
		// Thin shapes turn inside out when inset; keep those sharp.
		for (int i = 0; i < n; i++)
		{
			if (b2Dot(b2Sub(rounded.vertices[(i + 1) % n], rounded.vertices[i]), b2Sub(sharp.vertices[(i + 1) % n], sharp.vertices[i])) <= 0.0f)
			{
				return sharp;
			}
		}
		return rounded;
	}

	struct Run
	{
		int l, r; // first and last cell of a horizontal run
	};

	// Do straight lines through the ends of the band's first and last runs
	// stay within edgeTolerance of the ends of every run in between?
	bool Straight(const std::vector<Run> &runs)
	{
		auto k = int(runs.size()) - 1;
		if (k < 2)
		{
			return true;
		}
		for (int i = 1; i < k; i++)
		{
			auto t = float(i) / float(k);
			auto l = runs[0].l + (runs[k].l - runs[0].l) * t;
			auto r = runs[0].r + (runs[k].r - runs[0].r) * t;
			if (std::abs(l - runs[i].l) > edgeTolerance || std::abs(r - runs[i].r) > edgeTolerance)
			{
				return false;
			}
		}
		return true;
	}

	// Cover the set cells of a w*h bitmap with convex polygons whose sides
	// follow staircase edges as straight lines, so diagonal and curved edges
	// collide like smooth ones instead of catching on every step. Rows are
	// grouped into bands in which every row has a single run of cells that
	// continues the run above it and Straight holds; each band becomes a
	// trapezoid. A pixel covers the unit square centred on its coordinates.
	template<class Solid>
	std::vector<b2Polygon> SmoothPolygons(int w, int h, Solid solid, float offsetX, float offsetY)
	{
		struct Band
		{
			int y0;
			std::vector<Run> runs;
		};
		std::vector<b2Polygon> polygons;
		auto emit = [&](const Band &band) {
			auto &runs = band.runs;
			auto k = float(runs.size() - 1);
			// Extend the lines through the run ends half a row past the
			// first and last rows to reach the band's top and bottom edges.
			auto edges = [&](float first, float last) {
				auto slope = k > 0 ? (last - first) / k : 0.0f;
				return std::pair(first - slope * 0.5f, last + slope * 0.5f);
			};
			auto [lt, lb] = edges(runs.front().l - 0.5f, runs.back().l - 0.5f);
			auto [rt, rb] = edges(runs.front().r + 0.5f, runs.back().r + 0.5f);
			if (lt > rt)
			{
				lt = rt = (lt + rt) * 0.5f;
			}
			if (lb > rb)
			{
				lb = rb = (lb + rb) * 0.5f;
			}
			auto yt = offsetY + band.y0 - 0.5f, yb = offsetY + band.y0 + k + 0.5f;
			b2Vec2 points[] = {
				{ offsetX + lt, yt },
				{ offsetX + rt, yt },
				{ offsetX + rb, yb },
				{ offsetX + lb, yb },
			};
			auto hull = b2ComputeHull(points, 4);
			if (hull.count >= 3)
			{
				polygons.push_back(Rounded(b2MakePolygon(&hull, 0.0f)));
				return;
			}
			for (int i = 0; i < int(runs.size()); i++)
			{
				polygons.push_back(Rounded(b2MakeOffsetBox((runs[i].r - runs[i].l + 1) * 0.5f, 0.5f, {
					offsetX + (runs[i].l + runs[i].r) * 0.5f,
					offsetY + band.y0 + i,
				}, b2Rot_identity)));
			}
		};

		std::vector<Band> open;
		std::vector<Run> runs;
		for (int y = 0; y < h; y++)
		{
			runs.clear();
			for (int x = 0; x < w; x++)
			{
				if (solid(x, y))
				{
					auto l = x;
					while (x + 1 < w && solid(x + 1, y))
					{
						x++;
					}
					runs.push_back({ l, x });
				}
			}
			auto overlaps = [](const Run &a, const Run &b) {
				return a.l <= b.r && b.l <= a.r;
			};
			// Work out which band each run could continue before any band
			// is moved on to the next row.
			std::vector<int> bandRuns(open.size(), 0);
			std::vector<int> runBand(runs.size(), -1);
			for (int j = 0; j < int(runs.size()); j++)
			{
				int count = 0;
				for (int i = 0; i < int(open.size()); i++)
				{
					if (overlaps(open[i].runs.back(), runs[j]))
					{
						bandRuns[i]++;
						runBand[j] = i;
						count++;
					}
				}
				if (count != 1)
				{
					runBand[j] = -1;
				}
			}
			std::vector<bool> continued(open.size(), false);
			std::vector<Band> next;
			for (int j = 0; j < int(runs.size()); j++)
			{
				auto &run = runs[j];
				auto band = runBand[j];
				if (band != -1 && bandRuns[band] == 1)
				{
					open[band].runs.push_back(run);
					if (Straight(open[band].runs))
					{
						continued[band] = true;
						next.push_back(std::move(open[band]));
						continue;
					}
					open[band].runs.pop_back();
				}
				next.push_back({ y, { run } });
			}
			for (int i = 0; i < int(open.size()); i++)
			{
				if (!continued[i])
				{
					emit(open[i]);
				}
			}
			open = std::move(next);
		}
		for (auto &band : open)
		{
			emit(band);
		}
		return polygons;
	}
}

RigidBodies::RigidBodies() : lastState(NPART, noParticle), partBody(NPART, 0), partIndex(NPART, -1), claimStamp(XRES * YRES, 0), claimBody(XRES * YRES, 0), visited(XRES * YRES, 0), sideStamp(XRES * YRES, 0), side(XRES * YRES, 0)
{
	// Box2D is tuned for bodies about 0.1 to 10 units across. With this, one
	// unit is one pixel and a 10 pixel body counts as a metre.
	b2SetLengthUnitsPerMeter(10.0f);
	auto worldDef = b2DefaultWorldDef();
	// Gravity is applied per body so that every TPT gravity mode works.
	worldDef.gravity = { 0.0f, 0.0f };
	// Stiffer contacts than Box2D's default, so that a heavy body resting on
	// a light one (gold cargo on a wooden floor) doesn't sink into it: its
	// pixels can only follow it there by overlapping the other body's.
	// Box2D allows up to an eighth of the substep rate.
	worldDef.contactHertz = 0.125f * frameRate * subSteps;
	world = b2CreateWorld(&worldDef);
	auto terrainDef = b2DefaultBodyDef();
	terrain = b2CreateBody(world, &terrainDef);
}

RigidBodies::~RigidBodies()
{
	b2DestroyWorld(world);
}

void RigidBodies::Clear()
{
	for (auto &[id, body] : bodies)
	{
		b2DestroyBody(body.b2);
	}
	bodies.clear();
	for (auto &tile : tiles)
	{
		for (auto shape : tile.shapes)
		{
			b2DestroyShape(shape, false);
		}
		tile = Tile();
	}
	std::fill(partBody.begin(), partBody.end(), 0);
	std::fill(lastState.begin(), lastState.end(), noParticle);
	lastActive = 0;
}

std::pair<b2Vec2, b2Vec2> RigidBodies::PoseOf(int id) const
{
	auto it = bodies.find(id);
	if (it == bodies.end())
	{
		return {};
	}
	return { b2Body_GetPosition(it->second.b2), b2Body_GetLinearVelocity(it->second.b2) };
}

std::vector<int> RigidBodies::BodiesIn(const Simulation &sim, const std::vector<std::pair<int, int>> &cells) const
{
	std::vector<int> ids;
	for (auto [x, y] : cells)
	{
		if (x < 0 || x >= XRES || y < 0 || y >= YRES)
		{
			continue;
		}
		auto r = sim.pmap[y][x];
		auto id = r ? partBody[ID(r)] : 0;
		if (id && bodies.count(id) && std::find(ids.begin(), ids.end(), id) == ids.end())
		{
			ids.push_back(id);
		}
	}
	return ids;
}

void RigidBodies::Push(const Simulation &sim, const std::vector<std::pair<int, int>> &cells, b2Vec2 at, b2Vec2 speed)
{
	for (auto id : BodiesIn(sim, cells))
	{
		auto &body = bodies.at(id);
		// Its mass if it were water: its size, in pixels.
		auto size = float(body.pixels.size());
		b2Body_ApplyLinearImpulse(body.b2, b2MulSV(size * frameRate, speed), at, true);
	}
}

void RigidBodies::Poke(const Simulation &sim, const std::vector<std::pair<int, int>> &cells, b2Vec2 at, float speed)
{
	for (auto id : BodiesIn(sim, cells))
	{
		auto &body = bodies.at(id);
		auto away = b2Sub(b2Body_GetWorldCenterOfMass(body.b2), at);
		// Poked right in the middle, it goes up, against gravity.
		if (b2Length(away) < 1.0f)
		{
			away = b2Length(body.gravity) > 1e-6f ? b2Neg(body.gravity) : b2Vec2{ 0.0f, -1.0f };
		}
		auto size = float(body.pixels.size());
		b2Body_ApplyLinearImpulse(body.b2, b2MulSV(size * frameRate * speed, b2Normalize(away)), at, true);
	}
}

void RigidBodies::Update(Simulation &sim)
{
	auto time = [](double &total, auto &&phase) {
		auto start = std::chrono::steady_clock::now();
		phase();
		total += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
	};
	frame++;
	time(timings.gather, [&] { Gather(sim); });
	MoveGrains(sim);
	if (bodies.empty())
	{
		return;
	}
	time(timings.terrain, [&] { RefreshTerrain(sim); });
	time(timings.step, [&] {
		ApplyGravity(sim);
		ApplyFluidForces(sim);
		ApplyAirForces(sim);
		b2World_Step(world, timeStep, subSteps);
	});
	time(timings.place, [&] {
		Place(sim);
		if (levelLiquids)
		{
			LevelLiquids(sim);
		}
	});
}

// Find every rigid pixel, keep intact bodies, and build new bodies out of
// pixels that are new, edited, pasted, or left over from damaged bodies.
void RigidBodies::Gather(Simulation &sim)
{
	auto &parts = sim.parts;
	for (auto &[id, body] : bodies)
	{
		body.alive = 0;
		for (auto &p : body.pixels)
		{
			p.alive = false;
			partBody[p.id] = 0;
		}
	}

	grains.clear();
	std::vector<Loose> loose;
	int lastTag = 0;
	Body *lastBody = nullptr;
	auto scanTo = std::max(parts.active, lastActive);
	for (int i = 0; i < scanTo; i++)
	{
		auto &part = parts[i];
		auto material = i < parts.active ? RigidMaterial(part) : 0;
		auto was = lastState[i];
		lastState[i] = i >= parts.active || !part.type ? noParticle : material ? rigidParticle : otherParticle;
		if (!material)
		{
			continue;
		}
		auto x = int(part.x + 0.5f);
		auto y = int(part.y + 0.5f);
		// Pixels of a body are usually next to each other in the particle list.
		if (!lastBody || part.tmp2 != lastTag)
		{
			auto it = bodies.find(part.tmp2);
			lastTag = part.tmp2;
			lastBody = it == bodies.end() ? nullptr : &it->second;
		}
		if (lastBody && lastBody->material == material)
		{
			auto &pixels = lastBody->pixels;
			auto index = partIndex[i];
			if (index >= 0 && index < int(pixels.size()))
			{
				auto &p = pixels[index];
				if (p.id == i && p.wx == x && p.wy == y && p.lx == part.tmp3 && p.ly == part.tmp4)
				{
					p.alive = true;
					lastBody->alive++;
					partBody[i] = lastBody->id;
					continue;
				}
			}
		}
		// Pixels from an earlier session or a stamp keep their tag so that
		// separate bodies stay separate; brand new ones have 0. Ones that
		// were something else (molten gold that has cooled, say) may sit
		// between cells; rigid pixels are always exactly on one.
		part.x = float(x);
		part.y = float(y);
		loose.push_back({ i, x, y, 0.0f, 0.0f, part.tmp2, material, was == otherParticle });
	}

	lastActive = parts.active;

	// Bodies that lost pixels (melted, cut, rusted...) may have split.
	std::vector<int> damaged;
	for (auto &[id, body] : bodies)
	{
		if (body.alive < int(body.pixels.size()))
		{
			damaged.push_back(id);
		}
	}
	std::sort(damaged.begin(), damaged.end());
	for (auto id : damaged)
	{
		Dissolve(bodies.at(id), loose, -id);
	}
	if (loose.empty())
	{
		return;
	}

	// Union-find over loose pixels. Pixels join if they touch, are the same
	// material and either share a tag (which keeps separate bodies in a save
	// separate) or one of them is new: freshly drawn (tag 0), or something
	// else last frame (molten gold that has just set). Grains that land on
	// each other stay separate grains.
	// Loose pixel index by cell, using the claim grid from Place as scratch.
	stamp++;
	for (int n = 0; n < int(loose.size()); n++)
	{
		auto k = loose[n].y * XRES + loose[n].x;
		claimStamp[k] = stamp;
		claimBody[k] = n;
	}
	std::vector<int> parent(loose.size());
	std::iota(parent.begin(), parent.end(), 0);
	auto find = [&parent](int n) {
		while (parent[n] != n)
		{
			parent[n] = parent[parent[n]];
			n = parent[n];
		}
		return n;
	};
	for (int n = 0; n < int(loose.size()); n++)
	{
		auto &p = loose[n];
		for (int ry = -1; ry <= 1; ry++)
		{
			for (int rx = -1; rx <= 1; rx++)
			{
				auto nx = p.x + rx, ny = p.y + ry;
				if (nx < 0 || nx >= XRES || ny < 0 || ny >= YRES || claimStamp[ny * XRES + nx] != stamp)
				{
					continue;
				}
				auto m = claimBody[ny * XRES + nx];
				auto &q = loose[m];
				if (m != n && p.material == q.material && (p.tag == q.tag || p.tag == 0 || q.tag == 0 || p.fresh || q.fresh))
				{
					parent[find(m)] = find(n);
				}
			}
		}
	}

	struct Group
	{
		std::vector<Loose> pixels;
		bool fresh = false;
		int material = 0;
		std::vector<int> absorbs; // IDs of intact bodies this group takes over
		int mergedInto = -1;
	};
	std::vector<Group> groups;
	std::unordered_map<int, int> groupOf; // union-find root -> group index
	for (int n = 0; n < int(loose.size()); n++)
	{
		auto root = find(n);
		auto [it, inserted] = groupOf.try_emplace(root, int(groups.size()));
		if (inserted)
		{
			groups.emplace_back();
		}
		auto &g = groups[it->second];
		g.pixels.push_back(loose[n]);
		g.fresh |= loose[n].tag == 0 || loose[n].fresh;
		g.material = loose[n].material;
	}

	// Freshly drawn or solidified pixels stick to whatever body of their
	// material they touch (molten metal setting on metal welds to it). One
	// brush stroke can leave several separate pieces around the same body
	// (say, a sliver on each side), so every piece touching a body has to end
	// up in the same new body as that body.
	auto final = [&groups](int g) {
		while (groups[g].mergedInto != -1)
		{
			g = groups[g].mergedInto;
		}
		return g;
	};
	std::unordered_map<int, int> owner; // intact body ID -> group that absorbs it
	for (int g = 0; g < int(groups.size()); g++)
	{
		if (!groups[g].fresh)
		{
			continue;
		}
		std::vector<int> touching;
		for (auto &p : groups[g].pixels)
		{
			for (int ry = -1; ry <= 1; ry++)
			{
				for (int rx = -1; rx <= 1; rx++)
				{
					auto nx = p.x + rx, ny = p.y + ry;
					if (nx < 0 || nx >= XRES || ny < 0 || ny >= YRES)
					{
						continue;
					}
					auto r = sim.pmap[ny][nx];
					auto id = r ? partBody[ID(r)] : 0;
					auto it = id ? bodies.find(id) : bodies.end();
					if (it != bodies.end() && it->second.material == groups[g].material && std::find(touching.begin(), touching.end(), id) == touching.end())
					{
						touching.push_back(id);
					}
				}
			}
		}
		for (auto id : touching)
		{
			auto mine = final(g);
			auto it = owner.find(id);
			if (it == owner.end())
			{
				owner[id] = mine;
				groups[mine].absorbs.push_back(id);
				continue;
			}
			auto theirs = final(it->second);
			if (theirs != mine)
			{
				// Fold this group into the one that already owns the body.
				auto &from = groups[mine];
				auto &to = groups[theirs];
				to.pixels.insert(to.pixels.end(), from.pixels.begin(), from.pixels.end());
				to.absorbs.insert(to.absorbs.end(), from.absorbs.begin(), from.absorbs.end());
				from.pixels.clear();
				from.absorbs.clear();
				from.mergedInto = theirs;
			}
		}
	}
	for (auto &g : groups)
	{
		if (g.mergedInto != -1)
		{
			continue;
		}
		for (auto id : g.absorbs)
		{
			auto it = bodies.find(id);
			if (it != bodies.end())
			{
				Dissolve(it->second, g.pixels, 0);
			}
		}
		// A lone pixel isn't worth a rigid body: it moves like a grain of
		// powder instead (see MoveGrains).
		if (int(g.pixels.size()) < minBodyPixels)
		{
			for (auto &p : g.pixels)
			{
				auto &part = sim.parts[p.id];
				part.tmp2 = grainTag;
				part.vx = 0;
				part.vy = 0;
				grains.push_back(p.id);
			}
			continue;
		}
		Bake(sim, g.pixels);
	}
}

// Lone rigid pixels fall like powder: a cell a frame along gravity, sliding
// off diagonally when blocked, sinking through liquid lighter than they are
// and rising through liquid heavier than they are.
void RigidBodies::MoveGrains(Simulation &sim)
{
	auto &elements = SimulationData::CRef().elements;
	for (auto id : grains)
	{
		auto &part = sim.parts[id];
		auto material = RigidMaterial(part);
		if (!material)
		{
			continue;
		}
		auto density = MaterialOf(material)->density;
		auto x = int(part.x + 0.5f), y = int(part.y + 0.5f);
		float gx, gy;
		sim.GetGravityField(x, y, 1.0f, 1.0f, gx, gy);
		if (std::abs(gx) < 1e-3f && std::abs(gy) < 1e-3f)
		{
			continue;
		}
		// Gravity's main direction, and the two diagonals either side of it.
		std::pair<int, int> down = std::abs(gy) >= std::abs(gx) ? std::pair(0, gy > 0 ? 1 : -1) : std::pair(gx > 0 ? 1 : -1, 0);
		std::pair<int, int> across = { down.second, down.first };
		auto side = sim.rng.chance(1, 2) ? 1 : -1;
		// Can the grain go into this cell, and does it swap with what's there?
		auto into = [&](int nx, int ny, bool rising) {
			if (nx < CELL || nx >= XRES - CELL || ny < CELL || ny >= YRES - CELL || sim.IsWallBlocking(nx, ny, material))
			{
				return false;
			}
			auto r = sim.pmap[ny][nx];
			if (!r)
			{
				return !rising;
			}
			if (!IsFluid(TYP(r)))
			{
				return false;
			}
			auto fluid = (elements[TYP(r)].Properties & TYPE_LIQUID) ? FluidDensity(TYP(r)) : 0.0f;
			return rising ? fluid > density : fluid < density;
		};
		auto go = [&](int nx, int ny) {
			auto r = sim.pmap[ny][nx];
			if (r)
			{
				auto &other = sim.parts[ID(r)];
				other.x = float(x);
				other.y = float(y);
			}
			sim.pmap[y][x] = r;
			sim.pmap[ny][nx] = PMAP(id, part.type);
			part.x = float(nx);
			part.y = float(ny);
		};
		std::pair<int, int> falls[] = {
			down,
			{ down.first + across.first * side, down.second + across.second * side },
			{ down.first - across.first * side, down.second - across.second * side },
		};
		bool moved = false;
		for (auto [dx, dy] : falls)
		{
			if (into(x + dx, y + dy, false))
			{
				go(x + dx, y + dy);
				moved = true;
				break;
			}
		}
		if (!moved && into(x - down.first, y - down.second, true))
		{
			go(x - down.first, y - down.second);
		}
	}
}

// Break a body back into loose pixels, each with its current velocity.
void RigidBodies::Dissolve(Body &body, std::vector<Loose> &into, int tag)
{
	auto v = b2Body_GetLinearVelocity(body.b2);
	auto w = b2Body_GetAngularVelocity(body.b2);
	auto c = b2Body_GetWorldCenterOfMass(body.b2);
	for (auto &p : body.pixels)
	{
		if (p.alive)
		{
			auto rx = p.wx - c.x, ry = p.wy - c.y;
			into.push_back({ p.id, p.wx, p.wy, v.x - w * ry, v.y + w * rx, tag, body.material });
		}
		partBody[p.id] = 0;
	}
	b2DestroyBody(body.b2);
	auto id = body.id;
	bodies.erase(id);
}

// Turn loose pixels into a new body at angle 0, keeping their combined
// momentum and spin.
void RigidBodies::Bake(Simulation &sim, std::vector<Loose> &pixels)
{
	auto n = float(pixels.size());
	float sx = 0, sy = 0, svx = 0, svy = 0;
	int minX = XRES, minY = YRES, maxX = -1, maxY = -1;
	for (auto &p : pixels)
	{
		sx += p.x;
		sy += p.y;
		svx += p.vx;
		svy += p.vy;
		minX = std::min(minX, p.x);
		minY = std::min(minY, p.y);
		maxX = std::max(maxX, p.x);
		maxY = std::max(maxY, p.y);
	}
	auto cx = sx / n, cy = sy / n;
	auto vx = svx / n, vy = svy / n;
	auto ox = Round(cx), oy = Round(cy);
	// Each pixel is a unit square: its own inertia about its centre is 1/6.
	auto inertia = n / 6.0f, spin = 0.0f;
	for (auto &p : pixels)
	{
		auto rx = p.x - cx, ry = p.y - cy;
		inertia += rx * rx + ry * ry;
		spin += rx * (p.vy - vy) - ry * (p.vx - vx);
	}

	auto bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = { float(ox), float(oy) };
	bodyDef.linearVelocity = { vx, vy };
	bodyDef.angularVelocity = spin / inertia;

	Body body;
	body.id = nextBodyId++;
	body.material = pixels.front().material;
	auto &material = *MaterialOf(body.material);
	body.b2 = b2CreateBody(world, &bodyDef);
	body.alive = int(pixels.size());
	body.gravity = { 0.0f, 0.0f };
	body.displaced = 0;
	body.drawnPos = bodyDef.position;
	body.drawnRot = b2Rot_identity;

	auto w = maxX - minX + 1, h = maxY - minY + 1;
	std::vector<bool> solid(w * h, false);
	for (auto &p : pixels)
	{
		solid[(p.y - minY) * w + (p.x - minX)] = true;
	}
	auto shapeDef = b2DefaultShapeDef();
	shapeDef.density = material.density;
	shapeDef.material.friction = material.friction;
	shapeDef.material.restitution = material.restitution;
	for (auto &polygon : SmoothPolygons(w, h, [&](int x, int y) { return bool(solid[y * w + x]); }, float(minX - ox), float(minY - oy)))
	{
		b2CreatePolygonShape(body.b2, &shapeDef, &polygon);
	}

	body.reach = 0;
	for (auto &p : pixels)
	{
		body.reach = std::max(body.reach, std::hypot(float(p.x - ox), float(p.y - oy)));
	}
	body.maskX = minX - ox;
	body.maskY = minY - oy;
	body.maskW = w;
	body.maskH = h;
	body.mask = std::move(solid);

	auto &parts = sim.parts;
	for (auto &p : pixels)
	{
		auto index = int(body.pixels.size());
		body.pixels.push_back({ p.id, p.x - ox, p.y - oy, p.x, p.y, true });
		partBody[p.id] = body.id;
		partIndex[p.id] = index;
		auto &part = parts[p.id];
		part.tmp2 = body.id;
		part.tmp3 = p.x - ox;
		part.tmp4 = p.y - oy;
		part.vx = 0;
		part.vy = 0;
	}
	bodies.emplace(body.id, std::move(body));
}

bool RigidBodies::TerrainSolid(const Simulation &sim, int x, int y) const
{
	if (x < CELL || x >= XRES - CELL || y < CELL || y >= YRES - CELL)
	{
		// With void edges the engine deletes pixels that leave the screen.
		return sim.edgeMode != EDGE_VOID;
	}
	if (sim.IsWallBlocking(x, y, PT_RIGD))
	{
		return true;
	}
	auto r = sim.pmap[y][x];
	return r && !partBody[ID(r)] && !IsFluid(TYP(r));
}

// Mirror walls and non-rigid particles near each body into Box2D as static
// boxes, in tiles that are only rebuilt when what is in them changes.
void RigidBodies::RefreshTerrain(Simulation &sim)
{
	struct Box
	{
		int x0, y0, x1, y1;
	};
	std::vector<Box> reach;
	std::vector<bool> wanted(tiles.size(), false);
	for (auto &[id, body] : bodies)
	{
		Box box{ XRES, YRES, -1, -1 };
		for (auto &p : body.pixels)
		{
			box.x0 = std::min(box.x0, p.wx);
			box.y0 = std::min(box.y0, p.wy);
			box.x1 = std::max(box.x1, p.wx);
			box.y1 = std::max(box.y1, p.wy);
		}
		auto v = b2Body_GetLinearVelocity(body.b2);
		auto margin = terrainMargin + int(std::ceil(2.0f * b2Length(v) * timeStep));
		box = { box.x0 - margin, box.y0 - margin, box.x1 + margin, box.y1 + margin };
		reach.push_back(box);
		// Sleeping bodies only need to notice their support changing, which
		// can wait a few frames; spread them out so they don't all check at once.
		if (!b2Body_IsAwake(body.b2) && (frame + id) % sleepingTerrainInterval)
		{
			continue;
		}
		for (int ty = std::max(0, box.y0 / tileSize); ty <= std::min(tilesY - 1, box.y1 / tileSize); ty++)
		{
			for (int tx = std::max(0, box.x0 / tileSize); tx <= std::min(tilesX - 1, box.x1 / tileSize); tx++)
			{
				wanted[ty * tilesX + tx] = true;
			}
		}
	}

	std::vector<Box> changed;
	for (int ty = 0; ty < tilesY; ty++)
	{
		for (int tx = 0; tx < tilesX; tx++)
		{
			auto &tile = tiles[ty * tilesX + tx];
			if (!wanted[ty * tilesX + tx])
			{
				continue;
			}
			auto x0 = tx * tileSize, y0 = ty * tileSize;
			auto w = std::min(tileSize, XRES - x0), h = std::min(tileSize, YRES - y0);
			decltype(tile.solid) solid;
			for (int y = 0; y < h; y++)
			{
				for (int x = 0; x < w; x++)
				{
					solid[y * tileSize + x] = TerrainSolid(sim, x0 + x, y0 + y);
				}
			}
			if (tile.built && solid == tile.solid)
			{
				continue;
			}
			for (auto shape : tile.shapes)
			{
				b2DestroyShape(shape, false);
			}
			tile.shapes.clear();
			auto shapeDef = b2DefaultShapeDef();
			shapeDef.material.friction = terrainFriction;
			shapeDef.material.restitution = 0.0f;
			for (auto &polygon : SmoothPolygons(w, h, [&](int x, int y) { return bool(solid[y * tileSize + x]); }, float(x0), float(y0)))
			{
				tile.shapes.push_back(b2CreatePolygonShape(terrain, &shapeDef, &polygon));
			}
			tile.solid = solid;
			tile.built = true;
			changed.push_back({ x0, y0, x0 + w - 1, y0 + h - 1 });
		}
	}

	// Removing support from under a sleeping body has to wake it up.
	int n = 0;
	for (auto &[id, body] : bodies)
	{
		auto &box = reach[n++];
		for (auto &c : changed)
		{
			if (box.x0 <= c.x1 && c.x0 <= box.x1 && box.y0 <= c.y1 && c.y0 <= box.y1)
			{
				b2Body_SetAwake(body.b2, true);
				break;
			}
		}
	}
}

void RigidBodies::ApplyGravity(Simulation &sim)
{
	for (auto &[id, body] : bodies)
	{
		auto c = b2Body_GetWorldCenterOfMass(body.b2);
		auto x = std::clamp(Round(c.x), 0, XRES - 1);
		auto y = std::clamp(Round(c.y), 0, YRES - 1);
		float gx, gy;
		sim.GetGravityField(x, y, 1.0f, 1.0f, gx, gy);
		// Changing the gravity mode has to wake bodies that are resting.
		if (std::abs(gx - body.gravity.x) + std::abs(gy - body.gravity.y) > 1e-3f)
		{
			body.gravity = { gx, gy };
			b2Body_SetAwake(body.b2, true);
		}
		if (!b2Body_IsAwake(body.b2))
		{
			continue;
		}
		auto m = b2Body_GetMass(body.b2) * gravityScale;
		b2Body_ApplyForceToCenter(body.b2, { gx * m, gy * m }, false);
	}
}

// Buoyancy and drag, a row of the body at a time. The liquid at either end
// of a row is what the row is displacing, so a row below the surface is
// pushed up by the weight of that much liquid, at the row's middle; that also
// turns floating bodies the right way up. Air enclosed within a row's span
// counts as displaced too, so hollow bodies (boats) float. A body touching
// liquid can't sleep, and a sleeping one is woken when liquid reaches it.
void RigidBodies::ApplyFluidForces(Simulation &sim)
{
	auto &elements = SimulationData::CRef().elements;
	std::vector<std::pair<int, int>> rows;
	for (auto &[id, body] : bodies)
	{
		auto awake = b2Body_IsAwake(body.b2);
		if (!awake && (frame + id) % sleepingTerrainInterval)
		{
			continue;
		}
		int y0 = YRES, y1 = -1;
		for (auto &p : body.pixels)
		{
			y0 = std::min(y0, p.wy);
			y1 = std::max(y1, p.wy);
		}
		if (y1 < y0)
		{
			continue;
		}
		rows.assign(y1 - y0 + 1, { XRES, -1 });
		for (auto &p : body.pixels)
		{
			auto &row = rows[p.wy - y0];
			row.first = std::min(row.first, p.wx);
			row.second = std::max(row.second, p.wx);
		}
		auto c = b2Body_GetWorldCenterOfMass(body.b2);
		float gx, gy;
		sim.GetGravityField(std::clamp(Round(c.x), 0, XRES - 1), std::clamp(Round(c.y), 0, YRES - 1), 1.0f, 1.0f, gx, gy);
		auto v = b2Body_GetLinearVelocity(body.b2);
		auto w = b2Body_GetAngularVelocity(body.b2);
		// Density of what's in a cell: liquid, air (0), or -1 if it's something
		// solid that says nothing about the liquid level.
		auto densityAt = [&](int x, int y) {
			if (x < 0 || x >= XRES)
			{
				return -1.0f;
			}
			auto r = sim.pmap[y][x];
			if (!r || (elements[TYP(r)].Properties & TYPE_GAS))
			{
				return sim.IsWallBlocking(x, y, PT_RIGD) ? -1.0f : 0.0f;
			}
			return IsFluid(TYP(r)) ? FluidDensity(TYP(r)) : -1.0f;
		};
		// What a row of the body is in, looking a few cells out from its end
		// (step is -1 or 1): right against the body the surface is uneven,
		// with liquid heaped up where it was pushed aside. Stops at anything
		// solid, so a body in a narrow gap still reads what's in the gap.
		auto beside = [&](int x, int y, int step) {
			auto density = -1.0f;
			for (int d = 0; d < fluidProbe; d++, x += step)
			{
				auto here = densityAt(x, y);
				if (here < 0)
				{
					break;
				}
				density = here;
			}
			return density;
		};
		// Each row's liquid density (0 if dry) and displaced cells.
		struct Row
		{
			float density = 0;
			int cells = 0;
			float sumX = 0;
		};
		std::vector<Row> info(y1 - y0 + 1);
		bool wet = false;
		for (int y = y0; y <= y1; y++)
		{
			auto [l, r] = rows[y - y0];
			if (r < l)
			{
				continue;
			}
			auto &row = info[y - y0];
			for (int x = l; x <= r; x++)
			{
				auto q = sim.pmap[y][x];
				if (!q || (q && partBody[ID(q)] == id) || (elements[TYP(q)].Properties & TYPE_GAS))
				{
					row.cells++;
					row.sumX += x;
				}
			}
			auto left = beside(l - 1, y, -1), right = beside(r + 1, y, 1);
			row.density = left >= 0 && right >= 0 ? (left + right) * 0.5f : std::max(left, right);
			wet |= row.density > 0 && row.cells;
		}
		// The rows are where the pixels are drawn, but Box2D's body is up to
		// placeThreshold away from there. How much lower (along gravity) a
		// point of the body is than where it's drawn:
		auto g = std::hypot(gx, gy);
		auto down = g > 1e-6f ? b2Vec2{ gx / g, gy / g } : b2Vec2{ 0.0f, 1.0f };
		auto drawn = b2Transform{ body.drawnPos, body.drawnRot };
		auto now = b2Body_GetTransform(body.b2);
		auto sunk = [&](b2Vec2 point) {
			return b2Dot(b2Sub(b2TransformPoint(now, b2InvTransformPoint(drawn, point)), point), down);
		};
		auto push = [&](const Row &row, int y, float density, float part) {
			b2Vec2 point{ row.sumX / row.cells, float(y) };
			auto weight = density * row.cells * part * gravityScale;
			auto pv = b2Vec2{ v.x - w * (point.y - c.y), v.y + w * (point.x - c.x) };
			auto drag = fluidDrag * density * row.cells * part;
			b2Vec2 force{ -gx * weight - drag * pv.x, -gy * weight - drag * pv.y };
			b2Body_ApplyForce(body.b2, force, point, true);
		};
		for (int y = y0; y <= y1; y++)
		{
			auto &row = info[y - y0];
			if (!row.cells)
			{
				continue;
			}
			auto above = y > y0 ? &info[y - y0 - 1] : nullptr;
			if (row.density > 0)
			{
				// The waterline row is only partly under: more of it the
				// further the body has sunk since it was drawn. That keeps the
				// force smooth, so a floating body settles rather than flipping
				// between two rows.
				auto part = 1.0f;
				if (!above || !above->cells || above->density <= 0)
				{
					part = std::clamp(1.0f + sunk({ row.sumX / row.cells, float(y) }), 0.0f, 1.0f);
					if (above && above->cells)
					{
						push(*above, y - 1, row.density, std::clamp(sunk({ above->sumX / above->cells, float(y - 1) }), 0.0f, 1.0f));
					}
				}
				push(row, y, row.density, part);
			}
		}
		b2Body_EnableSleep(body.b2, !wet);
	}
}

// Air currents and pressure, a face of the body at a time (see airDrag).
// Only differences in pressure across the body push it, and only air moving
// faster than calmAirSpeed drags it, so calm air leaves it alone. A body
// touching liquid gets its drag from that instead (see ApplyFluidForces). A
// body asleep is only woken if the air is strong enough to matter.
void RigidBodies::ApplyAirForces(Simulation &sim)
{
	auto &elements = SimulationData::CRef().elements;
	auto calm = [](float value, float threshold) {
		return std::abs(value) < threshold ? 0.0f : value - std::copysign(threshold, value);
	};
	for (auto &[id, body] : bodies)
	{
		auto awake = b2Body_IsAwake(body.b2);
		if (!awake && (frame + id) % sleepingTerrainInterval)
		{
			continue;
		}
		// Each face open to something other than the body: where it is, which
		// way it faces, and what's beyond it.
		struct Face
		{
			b2Vec2 arm;
			int dx, dy, nx, ny;
			bool air;
		};
		std::vector<Face> faces;
		auto c = b2Body_GetWorldCenterOfMass(body.b2);
		bool wet = false;
		float meanPressure = 0.0f;
		for (auto &p : body.pixels)
		{
			for (auto [dx, dy] : { std::pair(0, -1), { -1, 0 }, { 1, 0 }, { 0, 1 } })
			{
				auto nx = p.wx + dx, ny = p.wy + dy;
				if (nx < 0 || nx >= XRES || ny < 0 || ny >= YRES)
				{
					continue;
				}
				auto r = sim.pmap[ny][nx];
				if (r && partBody[ID(r)] == id)
				{
					continue;
				}
				auto properties = r ? elements[TYP(r)].Properties : 0;
				wet |= bool(properties & TYPE_LIQUID);
				faces.push_back({ { p.wx - c.x, p.wy - c.y }, dx, dy, nx, ny, !r || (properties & TYPE_GAS) });
				meanPressure += sim.pv[ny / CELL][nx / CELL];
			}
		}
		if (faces.empty())
		{
			continue;
		}
		meanPressure /= float(faces.size());
		auto v = b2Body_GetLinearVelocity(body.b2);
		auto w = b2Body_GetAngularVelocity(body.b2);
		b2Vec2 force{ 0.0f, 0.0f };
		float torque = 0.0f;
		for (auto &face : faces)
		{
			auto pressure = calm(sim.pv[face.ny / CELL][face.nx / CELL] - meanPressure, calmPressure);
			b2Vec2 f{ -airPressure * pressure * face.dx, -airPressure * pressure * face.dy };
			if (face.air)
			{
				auto ax = calm(sim.vx[face.ny / CELL][face.nx / CELL], calmAirSpeed);
				auto ay = calm(sim.vy[face.ny / CELL][face.nx / CELL], calmAirSpeed);
				f.x += airDrag * airSpeed * ax;
				f.y += airDrag * airSpeed * ay;
				if (!wet)
				{
					// The face's velocity, in pixels per frame.
					f.x -= airDrag * (v.x - w * face.arm.y) / frameRate;
					f.y -= airDrag * (v.y + w * face.arm.x) / frameRate;
				}
			}
			force = b2Add(force, f);
			torque += b2Cross(face.arm, f);
		}
		// To Box2D's units: per second rather than per frame, squared.
		force = b2MulSV(frameRate * frameRate, force);
		torque *= frameRate * frameRate;
		auto m = b2Body_GetMass(body.b2);
		if (!awake && b2Length(force) < 0.05f * m * gravityPerFrame * frameRate * frameRate)
		{
			continue;
		}
		b2Body_ApplyForceToCenter(body.b2, force, true);
		b2Body_ApplyTorque(body.b2, torque, true);
	}
}

// TPT's liquids have no pressure: they only flow into empty cells, so liquid
// heaped against one side of a floating body never flows under it to the
// other side, and the body acts like a dam. Make up for that: where the same
// liquid stands higher on one side of a floating body than the other, move a
// little from the top of the high side to the top of the low side each frame,
// as if it had flowed underneath.
void RigidBodies::LevelLiquids(Simulation &sim)
{
	auto &elements = SimulationData::CRef().elements;
	for (auto &[id, body] : bodies)
	{
		// The body's bottom row, and how far it reaches either way: levels
		// are compared beyond that, since a body heeled over overhangs the
		// liquid next to its lowest point.
		int y = -1, l = XRES, r = -1;
		for (auto &p : body.pixels)
		{
			y = std::max(y, p.wy);
			l = std::min(l, p.wx);
			r = std::max(r, p.wx);
		}
		if (y < 0 || y >= YRES - 1)
		{
			continue;
		}
		auto liquid = [&](int x, int yy) {
			if (x < 0 || x >= XRES || yy < 0)
			{
				return 0;
			}
			auto q = sim.pmap[yy][x];
			return q && (elements[TYP(q)].Properties & TYPE_LIQUID) ? TYP(q) : 0;
		};
		auto type = liquid(l - 1, y);
		if (!type || liquid(r + 1, y) != type)
		{
			continue;
		}
		// The top of the liquid standing in a column from the body's bottom
		// row up, or -1 if the column isn't that liquid at the bottom row.
		auto top = [&](int x) {
			if (liquid(x, y) != type)
			{
				return -1;
			}
			auto t = y;
			while (liquid(x, t - 1) == type)
			{
				t--;
			}
			return t;
		};
		// The columns beside one end of the body: the average height of the
		// liquid in them, and the columns where it stands highest and lowest.
		struct Side
		{
			float average = 0;
			int high = -1, highTop = 0, low = -1, lowTop = 0;
		};
		auto side = [&](int from, int step) {
			Side result;
			int count = 0;
			for (int d = 0, x = from; d < levelReach; d++, x += step)
			{
				auto t = top(x);
				if (t < 0)
				{
					break;
				}
				result.average += t;
				count++;
				if (result.high < 0 || t < result.highTop)
				{
					result.high = x;
					result.highTop = t;
				}
				if (result.low < 0 || t > result.lowTop)
				{
					result.low = x;
					result.lowTop = t;
				}
			}
			result.average = count ? result.average / count : 0.0f;
			return result;
		};
		for (int n = 0; n < maxLevelMoves; n++)
		{
			auto left = side(l - 1, -1), right = side(r + 1, 1);
			if (left.high < 0 || right.high < 0)
			{
				break;
			}
			// Compare averages: the surface right beside a body is uneven
			// from frame to frame, and that alone shouldn't move anything.
			auto leftToRight = left.average < right.average;
			auto &high = leftToRight ? left : right;
			auto &low = leftToRight ? right : left;
			if (low.average - high.average < 2.0f)
			{
				break;
			}
			auto from = high.high, fromTop = high.highTop;
			auto to = low.low, toTop = low.lowTop;
			if (toTop - 1 < 0 || sim.pmap[toTop - 1][to] || sim.IsWallBlocking(to, toTop - 1, type))
			{
				break;
			}
			auto q = sim.pmap[fromTop][from];
			auto &part = sim.parts[ID(q)];
			sim.pmap[fromTop][from] = 0;
			part.x = float(to);
			part.y = float(toTop - 1);
			sim.pmap[toTop - 1][to] = q;
			timings.fluidMoved++;
		}
	}
}

// Cells for every pixel of the body at the given pose, as x1, y1, x2, y2...
//
// The shape is decided first: every cell around the body is scored by how
// well the rotated body covers it, and the body takes exactly as many of the
// best covered cells as it has pixels. Cells well inside the body always come
// first, so the shape never has holes; only cells that the rotated edge barely
// reaches are left out or let in to make the count match. Each pixel then goes
// to the chosen cell nearest its exact position.
void RigidBodies::Place(const Body &body, b2Vec2 pos, b2Rot rot, std::vector<int> &cells)
{
	auto n = int(body.pixels.size());
	// A body turned so little from square that no pixel would be more than a
	// pixel out is drawn square. Drawn at its true angle, its sides would get
	// a one-pixel step in them, both at the same height, which looks like the
	// body has torn rather than turned; and two bodies resting together (cargo
	// in a box) rarely come to rest at quite the same tiny angle, so their
	// steps fall in different places, leaving slits and overlaps.
	{
		constexpr auto halfPi = std::numbers::pi_v<float> / 2.0f;
		auto angle = b2Rot_GetAngle(rot);
		auto quarter = std::round(angle / halfPi);
		if (std::abs(angle - quarter * halfPi) * body.reach < squareSnap)
		{
			rot = b2MakeRot(quarter * halfPi);
			rot = { std::round(rot.c), std::round(rot.s) };
		}
	}
	auto c = rot.c, s = rot.s;
	cells.resize(n * 2);
	int x0 = INT_MAX, y0 = INT_MAX, x1 = INT_MIN, y1 = INT_MIN;
	for (int i = 0; i < n; i++)
	{
		auto &p = body.pixels[i];
		auto x = Round(pos.x + c * p.lx - s * p.ly);
		auto y = Round(pos.y + s * p.lx + c * p.ly);
		cells[2 * i] = x;
		cells[2 * i + 1] = y;
		x0 = std::min(x0, x);
		y0 = std::min(y0, y);
		x1 = std::max(x1, x);
		y1 = std::max(y1, y);
	}
	x0 -= 2;
	y0 -= 2;
	auto w = x1 - x0 + 3, h = y1 - y0 + 3;
	auto in = [&](int x, int y) {
		return x >= x0 && x < x0 + w && y >= y0 && y < y0 + h;
	};
	auto at = [&](int x, int y) {
		return (y - y0) * w + (x - x0);
	};
	auto masked = [&](int lx, int ly) {
		lx -= body.maskX;
		ly -= body.maskY;
		return lx >= 0 && lx < body.maskW && ly >= 0 && ly < body.maskH && body.mask[ly * body.maskW + lx];
	};

	// Lower scores are better covered. Cells inside the body score -1 if
	// none of their body-local neighbours are outside the body, or else how
	// far the cell sits towards the outside (up to 0.5). Cells outside score
	// 1 plus how far they are from the nearest pixel of the body.
	placeScores.clear();
	static constexpr std::pair<int, int> around[] = { { 0, -1 }, { -1, 0 }, { 1, 0 }, { 0, 1 } };
	for (int y = y0; y < y0 + h; y++)
	{
		for (int x = x0; x < x0 + w; x++)
		{
			auto dx = x - pos.x, dy = y - pos.y;
			auto lx = c * dx + s * dy, ly = -s * dx + c * dy;
			auto mx = Round(lx), my = Round(ly);
			if (masked(mx, my))
			{
				auto score = -1.0f;
				for (auto [ax, ay] : around)
				{
					if (!masked(mx + ax, my + ay))
					{
						score = std::max(score, (lx - mx) * ax + (ly - my) * ay);
					}
				}
				placeScores.push_back({ score, at(x, y) });
				continue;
			}
			auto nearest = 9.0f;
			for (int ay = -1; ay <= 1; ay++)
			{
				for (int ax = -1; ax <= 1; ax++)
				{
					if (masked(mx + ax, my + ay))
					{
						nearest = std::min(nearest, std::max(std::abs(lx - (mx + ax)), std::abs(ly - (my + ay))));
					}
				}
			}
			if (nearest < 1.0f)
			{
				placeScores.push_back({ 1.0f + nearest, at(x, y) });
			}
		}
	}
	auto chosen = std::min(n, int(placeScores.size()));
	std::nth_element(placeScores.begin(), placeScores.begin() + chosen, placeScores.end());
	placeState.assign(w * h, 0); // 1: chosen, 2: chosen and taken
	for (int i = 0; i < chosen; i++)
	{
		placeState[placeScores[i].second] = 1;
	}

	// Tidy the outline: pixels sticking out with at most one neighbour fill
	// notches that have three. Rotating a pixelated edge leaves a few of both.
	auto neighbours = [&](int x, int y) {
		int count = 0;
		for (auto [ax, ay] : around)
		{
			count += in(x + ax, y + ay) && placeState[at(x + ax, y + ay)];
		}
		return count;
	};
	for (int pass = 0; pass < 3; pass++)
	{
		std::vector<std::pair<int, int>> notches, spurs;
		for (int y = y0; y < y0 + h; y++)
		{
			for (int x = x0; x < x0 + w; x++)
			{
				auto count = neighbours(x, y);
				if (!placeState[at(x, y)] && count >= 3)
				{
					notches.push_back({ x, y });
				}
				else if (placeState[at(x, y)] && count <= 1)
				{
					spurs.push_back({ x, y });
				}
			}
		}
		int swapped = 0;
		for (size_t i = 0, j = 0; i < notches.size() && j < spurs.size(); i++)
		{
			auto [nx, ny] = notches[i];
			if (placeState[at(nx, ny)] || neighbours(nx, ny) < 3)
			{
				continue;
			}
			while (j < spurs.size() && (!placeState[at(spurs[j].first, spurs[j].second)] || neighbours(spurs[j].first, spurs[j].second) > 1))
			{
				j++;
			}
			if (j == spurs.size())
			{
				break;
			}
			placeState[at(spurs[j].first, spurs[j].second)] = 0;
			placeState[at(nx, ny)] = 1;
			j++;
			swapped++;
		}
		if (!swapped)
		{
			break;
		}
	}

	std::vector<int> pending;
	for (int i = 0; i < n; i++)
	{
		auto k = at(cells[2 * i], cells[2 * i + 1]);
		if (placeState[k] == 1)
		{
			placeState[k] = 2;
		}
		else
		{
			pending.push_back(i);
		}
	}
	timings.placed += n;
	timings.pending += int(pending.size());
	// Each pixel that didn't land on a free chosen cell takes the nearest one
	// within a short search. Whatever is left is paired up in reading order;
	// the body looks the same either way.
	placeSeen.assign(w * h, 0);
	int seen = 0;
	std::vector<std::pair<int, int>> queue;
	std::vector<int> leftover;
	for (auto i : pending)
	{
		auto sx = cells[2 * i], sy = cells[2 * i + 1];
		seen++;
		queue.assign(1, { sx, sy });
		placeSeen[at(sx, sy)] = seen;
		std::optional<std::pair<int, int>> found;
		for (size_t head = 0; head < queue.size() && head < maxNearbySearch && !found; head++)
		{
			auto [cx, cy] = queue[head];
			timings.searchSteps++;
			for (auto [dx, dy] : around)
			{
				auto nx = cx + dx, ny = cy + dy;
				if (!in(nx, ny) || placeSeen[at(nx, ny)] == seen)
				{
					continue;
				}
				placeSeen[at(nx, ny)] = seen;
				if (placeState[at(nx, ny)] == 1)
				{
					found = std::pair(nx, ny);
					break;
				}
				queue.push_back({ nx, ny });
			}
		}
		if (!found)
		{
			leftover.push_back(i);
			continue;
		}
		cells[2 * i] = found->first;
		cells[2 * i + 1] = found->second;
		placeState[at(found->first, found->second)] = 2;
	}
	if (leftover.empty())
	{
		return;
	}
	std::sort(leftover.begin(), leftover.end(), [&](int a, int b) {
		return std::pair(cells[2 * a + 1], cells[2 * a]) < std::pair(cells[2 * b + 1], cells[2 * b]);
	});
	size_t next = 0;
	for (int y = y0; y < y0 + h && next < leftover.size(); y++)
	{
		for (int x = x0; x < x0 + w && next < leftover.size(); x++)
		{
			if (placeState[at(x, y)] == 1)
			{
				auto i = leftover[next++];
				cells[2 * i] = x;
				cells[2 * i + 1] = y;
				placeState[at(x, y)] = 2;
			}
		}
	}
	// Only if there were fewer candidate cells than pixels, which the margin
	// around the body should rule out: use any free cell.
	for (int y = y0; y < y0 + h && next < leftover.size(); y++)
	{
		for (int x = x0; x < x0 + w && next < leftover.size(); x++)
		{
			if (placeState[at(x, y)] == 0)
			{
				auto i = leftover[next++];
				cells[2 * i] = x;
				cells[2 * i + 1] = y;
				placeState[at(x, y)] = 2;
			}
		}
	}
}

// Move every body's pixels to its new pose. Rotated bodies are jagged, so
// where Box2D has two bodies (or a body and the terrain) touching, their
// pixels can overlap by a cell here and there. A body is shifted by a pixel
// if that helps, and any pixels still in the way are put in a free cell next
// to where they belong. Only a body that would need too many of those stays
// where it is drawn, and Box2D is pulled back to it if it drifts too far
// from it, keeping its velocity.
void RigidBodies::Place(Simulation &sim)
{
	struct Plan
	{
		Body *body;
		b2Vec2 fromPos, toPos;
		float fromAngle, turn;
		int steps;
		bool stuck = false;
	};
	std::vector<int> ids;
	for (auto &[id, body] : bodies)
	{
		ids.push_back(id);
	}
	std::sort(ids.begin(), ids.end());

	std::vector<Plan> plans;
	int most = 0;
	for (auto id : ids)
	{
		auto &body = bodies.at(id);
		auto v = b2Body_GetLinearVelocity(body.b2);
		if (b2Length(v) > maxSpeed)
		{
			b2Body_SetLinearVelocity(body.b2, b2MulSV(maxSpeed / b2Length(v), v));
		}
		auto pos = b2Body_GetPosition(body.b2);
		auto rot = b2Body_GetRotation(body.b2);
		// A body at rest with pixels moved aside keeps trying to put them
		// back, so the bulges heal once whatever pushed them is gone.
		auto turn = b2RelativeAngle(rot, body.drawnRot);
		auto moved = b2Distance(pos, body.drawnPos) + std::abs(turn) * body.reach;
		if (moved < placeThreshold && !body.displaced)
		{
			continue;
		}
		// Liquid doesn't stop bodies in Box2D, so a body moving through it
		// has to sweep through every cell on the way rather than jump to its
		// new place; otherwise liquid it skips over ends up inside it (in a
		// hollow body) or behind it. Elsewhere one step will do.
		auto steps = 1;
		if (moved > maxPlaceStep && FluidNear(sim, body, pos, moved))
		{
			steps = std::min(maxPlaceSteps, int(std::ceil(moved / maxPlaceStep)));
		}
		plans.push_back({ &body, body.drawnPos, pos, b2Rot_GetAngle(body.drawnRot), turn, steps });
		most = std::max(most, steps);
	}
	for (int k = 1; k <= most; k++)
	{
		std::vector<PlaceTarget> targets;
		for (auto &plan : plans)
		{
			if (plan.stuck || k > plan.steps)
			{
				continue;
			}
			auto t = float(k) / float(plan.steps);
			targets.push_back({ plan.body, b2Lerp(plan.fromPos, plan.toPos, t), b2MakeRot(plan.fromAngle + plan.turn * t), &plan.stuck });
		}
		PlaceStep(sim, targets);
	}

	for (auto &plan : plans)
	{
		if (!plan.stuck)
		{
			continue;
		}
		auto &body = *plan.body;
		timings.stuck++;
		auto pos = b2Body_GetPosition(body.b2);
		auto rot = b2Body_GetRotation(body.b2);
		auto drift = b2Distance(pos, body.drawnPos);
		auto turn = std::abs(b2RelativeAngle(body.drawnRot, rot));
		auto reach = body.reach;
		if (drift + turn * reach > maxDrift)
		{
			// Whatever stopped it is part of the terrain by the next frame,
			// so Box2D can deal with it properly from here. It hit something,
			// so it loses the speed it had towards it; otherwise it would
			// just go straight back in.
			timings.pulled++;
			b2Body_SetTransform(body.b2, body.drawnPos, body.drawnRot);
			auto towards = b2Sub(pos, body.drawnPos);
			if (b2Length(towards) > 1e-3f)
			{
				auto n = b2Normalize(towards);
				auto v = b2Body_GetLinearVelocity(body.b2);
				auto into = b2Dot(v, n);
				if (into > 0)
				{
					b2Body_SetLinearVelocity(body.b2, b2MulSub(v, into, n));
				}
			}
			if (turn * reach > drift)
			{
				b2Body_SetAngularVelocity(body.b2, b2Body_GetAngularVelocity(body.b2) * 0.5f);
			}
		}
	}
}

// Is there liquid or gas anywhere the body could pass through moving to pos?
bool RigidBodies::FluidNear(const Simulation &sim, const Body &body, b2Vec2 pos, float moved) const
{
	auto margin = body.reach + moved + 1.0f;
	auto x0 = std::max(0, int(std::floor(std::min(pos.x, body.drawnPos.x) - margin)));
	auto x1 = std::min(XRES - 1, int(std::ceil(std::max(pos.x, body.drawnPos.x) + margin)));
	auto y0 = std::max(0, int(std::floor(std::min(pos.y, body.drawnPos.y) - margin)));
	auto y1 = std::min(YRES - 1, int(std::ceil(std::max(pos.y, body.drawnPos.y) + margin)));
	for (int y = y0; y <= y1; y++)
	{
		for (int x = x0; x <= x1; x++)
		{
			auto r = sim.pmap[y][x];
			if (r && IsFluid(TYP(r)))
			{
				return true;
			}
		}
	}
	return false;
}

// Move bodies' pixels one step, to the given poses. Rotated bodies are
// jagged, so where Box2D has two bodies (or a body and the terrain) touching,
// their pixels can overlap by a cell here and there. A body is shifted by a
// pixel if that helps, and any pixels still in the way are put in a free cell
// next to where they belong. A body that would need too many of those stays
// where it is drawn and is marked stuck.
void RigidBodies::PlaceStep(Simulation &sim, const std::vector<PlaceTarget> &targets)
{
	struct Move
	{
		Body *body;
		b2Vec2 pos;
		b2Rot rot;
		bool *stuck;
		std::vector<int> target; // cells at the new pose, as x1, y1, x2, y2...
		std::vector<int> cells;  // cells actually used
		std::pair<int, int> up;
		int displaced = 0;
		bool stays = false;
		bool first = false; // moved to the front after failing to fit
	};
	std::vector<Move> moves;
	for (auto &step : targets)
	{
		auto &body = *step.body;
		Move move{ &body, step.pos, step.rot, step.stuck };
		Place(body, step.pos, step.rot, move.target);
		bool same = true;
		for (int n = 0; n < int(body.pixels.size()) && same; n++)
		{
			same = move.target[2 * n] == body.pixels[n].wx && move.target[2 * n + 1] == body.pixels[n].wy;
		}
		if (same)
		{
			body.drawnPos = step.pos;
			body.drawnRot = step.rot;
			body.displaced = 0;
			continue;
		}
		auto c = b2Body_GetWorldCenterOfMass(body.b2);
		float gx, gy;
		sim.GetGravityField(std::clamp(Round(c.x), 0, XRES - 1), std::clamp(Round(c.y), 0, YRES - 1), 1.0f, 1.0f, gx, gy);
		auto g = std::max(1e-6f, std::hypot(gx, gy));
		move.up = { -Round(gx / g), -Round(gy / g) };
		if (move.up == std::pair(0, 0))
		{
			move.up = { 0, -1 };
		}
		moves.push_back(std::move(move));
	}
	if (moves.empty())
	{
		return;
	}

	// Lift every moving body off the grid, then see where each one fits.
	for (auto &move : moves)
	{
		for (auto &p : move.body->pixels)
		{
			auto &r = sim.pmap[p.wy][p.wx];
			if (r && ID(r) == p.id)
			{
				r = 0;
			}
		}
	}
	auto outside = [](int x, int y) {
		return x < CELL || x >= XRES - CELL || y < CELL || y >= YRES - CELL;
	};
	// Is the cell unavailable to this body? Cells off the edge are available
	// with void edges, where the engine deletes pixels that leave the screen.
	auto taken = [&](int x, int y, int id) {
		if (outside(x, y))
		{
			return sim.edgeMode != EDGE_VOID;
		}
		auto k = y * XRES + x;
		auto r = sim.pmap[y][x];
		return (r && !IsFluid(TYP(r))) || sim.IsWallBlocking(x, y, PT_RIGD) || (claimStamp[k] == stamp && claimBody[k] != id);
	};
	auto claim = [&](int x, int y, int id) {
		if (!outside(x, y))
		{
			claimStamp[y * XRES + x] = stamp;
			claimBody[y * XRES + x] = id;
		}
	};
	auto fit = [&](Move &move) {
		auto id = move.body->id;
		auto n = int(move.target.size()) / 2;
		std::pair<int, int> nudges[] = {
			{ 0, 0 }, move.up,
			{ -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
			{ -1, -1 }, { 1, -1 }, { -1, 1 }, { 1, 1 },
		};
		std::pair<int, int> best;
		auto fewest = n + 1;
		for (auto nudge : nudges)
		{
			int count = 0;
			for (int i = 0; i < n && count < fewest; i++)
			{
				count += taken(move.target[2 * i] + nudge.first, move.target[2 * i + 1] + nudge.second, id);
			}
			if (count < fewest)
			{
				fewest = count;
				best = nudge;
			}
			if (!fewest)
			{
				break;
			}
		}
		// Only a few pixels may be moved aside: a body must look rigid, and
		// moving many of its pixels aside tears it. Overlapping more than that
		// is something Box2D doesn't know about yet, or bodies sunk into each
		// other; the body stays where it's drawn until Box2D sorts it out.
		if (fewest > maxDisplaced)
		{
			return false;
		}
		move.cells = move.target;
		std::vector<int> displaced;
		for (int i = 0; i < n; i++)
		{
			auto &x = move.cells[2 * i], &y = move.cells[2 * i + 1];
			x += best.first;
			y += best.second;
			if (taken(x, y, id))
			{
				displaced.push_back(i);
			}
			else
			{
				claim(x, y, id);
			}
		}
		// Each pixel that is in the way goes to the nearest free cell on the
		// body's own surface, searching only through its own cells: never
		// through another body, which would put it on the other side of that
		// body (cargo poking out through the hull of a ship it's in).
		for (auto i : displaced)
		{
			auto &x = move.cells[2 * i], &y = move.cells[2 * i + 1];
			visitStamp++;
			std::vector<int> queue{ y * XRES + x };
			visited[queue[0]] = visitStamp;
			bool found = false;
			for (size_t head = 0; head < queue.size() && head < maxSearch && !found; head++)
			{
				auto cx = queue[head] % XRES, cy = queue[head] / XRES;
				for (auto [dx, dy] : { std::pair(0, -1), { -1, 0 }, { 1, 0 }, { 0, 1 } })
				{
					auto nx = cx + dx, ny = cy + dy;
					if (outside(nx, ny))
					{
						continue;
					}
					auto k = ny * XRES + nx;
					if (visited[k] == visitStamp)
					{
						continue;
					}
					visited[k] = visitStamp;
					auto own = claimStamp[k] == stamp && claimBody[k] == id;
					if (!own && !taken(nx, ny, id))
					{
						x = nx;
						y = ny;
						claim(x, y, id);
						found = true;
						break;
					}
					if (own)
					{
						queue.push_back(k);
					}
				}
			}
			if (!found)
			{
				return false;
			}
		}
		move.displaced = int(displaced.size());
		return true;
	};

	// A body that doesn't fit gets one more try with first pick of the cells,
	// so where two bodies overlap it's the other one that gives way: one boxed
	// in with no free surface of its own (cargo filling a cabin) can't move
	// pixels aside, but the box around it can. If it still doesn't fit, it
	// stays where it is, keeping its old cells, which can take cells that
	// other bodies were given, so start over whenever that happens.
	for (bool again = true; again; )
	{
		again = false;
		stamp++;
		for (auto &move : moves)
		{
			if (move.stays)
			{
				for (auto &p : move.body->pixels)
				{
					claim(p.wx, p.wy, move.body->id);
				}
			}
		}
		for (auto it = moves.begin(); it != moves.end(); ++it)
		{
			if (it->stays || fit(*it))
			{
				continue;
			}
			if (!it->first)
			{
				it->first = true;
				std::rotate(moves.begin(), it, it + 1);
			}
			else
			{
				it->stays = true;
			}
			again = true;
			break;
		}
	}

	auto &parts = sim.parts;
	// Liquid and gas particles each moving body pushes out of the way, and
	// the cells it leaves.
	std::vector<std::vector<int>> pushed(moves.size()), left(moves.size());
	for (size_t m = 0; m < moves.size(); m++)
	{
		auto &move = moves[m];
		auto &body = *move.body;
		for (int n = 0; n < int(body.pixels.size()); n++)
		{
			auto &p = body.pixels[n];
			if (!move.stays)
			{
				auto x = move.cells[2 * n], y = move.cells[2 * n + 1];
				left[m].push_back(p.wy * XRES + p.wx);
				if (outside(x, y))
				{
					sim.kill_part(p.id);
					continue;
				}
				p.wx = x;
				p.wy = y;
			}
			auto &r = sim.pmap[p.wy][p.wx];
			if (r && ID(r) != p.id && IsFluid(TYP(r)))
			{
				timings.fluidMoved++;
				pushed[m].push_back(ID(r));
			}
			auto &part = parts[p.id];
			part.x = float(p.wx);
			part.y = float(p.wy);
			r = PMAP(p.id, part.type);
		}
		if (!move.stays)
		{
			body.drawnPos = move.pos;
			body.drawnRot = move.rot;
			body.displaced = move.displaced;
		}
		else
		{
			*move.stuck = true;
		}
	}

	// Pushed liquid and gas goes where it would flow, and always on its own
	// side of the body: water in a hollow stays in, and the liquid round it
	// stays out. Cells the body left that are surrounded by liquid (it's
	// moving through it) are filled first, matched up across the direction it
	// moved so the liquid closes in behind it side by side. The rest rises
	// through the liquid to the surface, so the level goes up as a body sinks
	// in. Failing that it takes a free cell nearby that it can get to without
	// passing through a rigid body.
	auto liquidAt = [&](int x, int y) {
		auto r = outside(x, y) ? 0 : sim.pmap[y][x];
		return r && IsFluid(TYP(r));
	};
	auto put = [&](int id, int k) {
		auto &part = parts[id];
		part.x = float(k % XRES);
		part.y = float(k / XRES);
		sim.pmap[k / XRES][k % XRES] = PMAP(id, part.type);
	};
	// Which side of a body each cell near it is on: cells that liquid could
	// flow between share a side, and a closed hollow is a side of its own.
	// Everything beyond a small margin round the body is outside (side 1).
	auto flows = [&](int x, int y) {
		auto r = sim.pmap[y][x];
		return !r || IsFluid(TYP(r));
	};
	int bx0 = 0, by0 = 0, bx1 = -1, by1 = -1;
	auto inBox = [&](int x, int y) {
		return x >= bx0 && x <= bx1 && y >= by0 && y <= by1;
	};
	auto sideOf = [&](int k) {
		return sideStamp[k] == sidePass ? side[k] : 1;
	};
	auto findSides = [&](size_t m) {
		bx0 = XRES;
		by0 = YRES;
		bx1 = by1 = -1;
		auto extend = [&](int x, int y) {
			bx0 = std::min(bx0, x);
			by0 = std::min(by0, y);
			bx1 = std::max(bx1, x);
			by1 = std::max(by1, y);
		};
		for (auto &p : moves[m].body->pixels)
		{
			extend(p.wx, p.wy);
		}
		for (auto k : left[m])
		{
			extend(k % XRES, k / XRES);
		}
		bx0 = std::max(CELL, bx0 - 2);
		by0 = std::max(CELL, by0 - 2);
		bx1 = std::min(XRES - CELL - 1, bx1 + 2);
		by1 = std::min(YRES - CELL - 1, by1 + 2);
		sidePass++;
		std::vector<int> queue;
		auto start = [&](int x, int y, int label) {
			auto k = y * XRES + x;
			if (sideStamp[k] != sidePass && flows(x, y))
			{
				sideStamp[k] = sidePass;
				side[k] = label;
				queue.push_back(k);
			}
		};
		auto fill = [&](int label) {
			for (size_t head = 0; head < queue.size(); head++)
			{
				auto cx = queue[head] % XRES, cy = queue[head] / XRES;
				for (auto [dx, dy] : { std::pair(0, -1), { -1, 0 }, { 1, 0 }, { 0, 1 } })
				{
					if (inBox(cx + dx, cy + dy))
					{
						start(cx + dx, cy + dy, label);
					}
				}
			}
			queue.clear();
		};
		for (int x = bx0; x <= bx1; x++)
		{
			start(x, by0, 1);
			start(x, by1, 1);
		}
		for (int y = by0; y <= by1; y++)
		{
			start(bx0, y, 1);
			start(bx1, y, 1);
		}
		fill(1);
		int next = 2;
		for (int y = by0; y <= by1; y++)
		{
			for (int x = bx0; x <= bx1; x++)
			{
				if (sideStamp[y * XRES + x] != sidePass && flows(x, y))
				{
					start(x, y, next);
					fill(next++);
				}
			}
		}
	};
	// Find places for pushed particles that didn't get a cell the body left:
	// one breadth-first search out from all of their old cells at once (now
	// the body's), stepping only through cells `through` allows. With the
	// particles' side known, only cells on that side are taken, and an empty
	// cell right beside one counts (it could have flowed there anyway).
	// Without it, it never steps straight from an old cell into an empty one
	// (that could mean passing through the body, into a hollow for instance).
	// Once it has found enough empty cells it looks a little further, and the
	// lowest ones win: liquid seeks its level, which stands in for the
	// pressure TPT's liquids don't have. Returns the particles it couldn't
	// place.
	auto findPlaces = [&](std::vector<int> unplaced, auto through, int on) {
		if (unplaced.empty())
		{
			return unplaced;
		}
		auto wall = [&](int x, int y) {
			return sim.IsWallBlocking(x, y, parts[unplaced[0]].type);
		};
		visitStamp++;
		std::vector<int> queue, found;
		for (auto id : unplaced)
		{
			auto sx = int(parts[id].x + 0.5f), sy = int(parts[id].y + 0.5f);
			visited[sy * XRES + sx] = visitStamp;
		}
		for (auto id : unplaced)
		{
			auto sx = int(parts[id].x + 0.5f), sy = int(parts[id].y + 0.5f);
			for (auto [dx, dy] : { std::pair(0, -1), { -1, 0 }, { 1, 0 }, { 0, 1 } })
			{
				auto nx = sx + dx, ny = sy + dy;
				auto k = ny * XRES + nx;
				if (outside(nx, ny) || visited[k] == visitStamp || (on && flows(nx, ny) && sideOf(k) != on))
				{
					continue;
				}
				if (!sim.pmap[ny][nx])
				{
					if (on && !wall(nx, ny))
					{
						visited[k] = visitStamp;
						found.push_back(k);
					}
				}
				else if (through(nx, ny))
				{
					visited[k] = visitStamp;
					queue.push_back(k);
				}
			}
		}
		size_t stopAt = found.size() >= unplaced.size() ? levelSearch : maxFluidSearch;
		for (size_t head = 0; head < queue.size() && head < stopAt; head++)
		{
			auto cx = queue[head] % XRES, cy = queue[head] / XRES;
			for (auto [dx, dy] : { std::pair(0, -1), { -1, 0 }, { 1, 0 }, { 0, 1 } })
			{
				auto nx = cx + dx, ny = cy + dy;
				auto k = ny * XRES + nx;
				if (outside(nx, ny) || visited[k] == visitStamp)
				{
					continue;
				}
				visited[k] = visitStamp;
				if (!sim.pmap[ny][nx])
				{
					if (!wall(nx, ny) && (!on || sideOf(k) == on))
					{
						found.push_back(k);
						if (found.size() == unplaced.size())
						{
							stopAt = std::min(maxFluidSearch, head + levelSearch);
						}
					}
					continue;
				}
				if (through(nx, ny))
				{
					queue.push_back(k);
				}
			}
		}
		std::stable_sort(found.begin(), found.end(), [](int a, int b) {
			return a / XRES > b / XRES;
		});
		auto placed = std::min(found.size(), unplaced.size());
		for (size_t i = 0; i < placed; i++)
		{
			put(unplaced[i], found[i]);
		}
		unplaced.erase(unplaced.begin(), unplaced.begin() + placed);
		return unplaced;
	};
	auto notRigid = [&](int x, int y) {
		auto r = sim.pmap[y][x];
		return r && !partBody[ID(r)];
	};
	auto anywhere = [](int, int) {
		return true;
	};
	// A particle with no room anywhere on its own side (a hollow filled to
	// the brim, which the body's outline squeezes a little as it turns)
	// takes the body's cell nearest to it on the edge of that side, and the
	// body's pixel there moves aside instead, to the nearest free cell on the
	// body's surface off that side. The body bulges by a pixel for a while,
	// like when it's pushed against something, and nothing is lost.
	auto makeRoom = [&](int id, int on) {
		auto x = int(parts[id].x + 0.5f), y = int(parts[id].y + 0.5f);
		auto r = sim.pmap[y][x];
		if (!r || !partBody[ID(r)])
		{
			return false;
		}
		auto owner = partBody[ID(r)];
		// Breadth-first through the body's own cells from `from`, to the
		// first one that `done` accepts (given the cell and the neighbouring
		// cell that isn't the body's), or -1.
		auto search = [&](int from, auto done) {
			visitStamp++;
			visited[from] = visitStamp;
			std::vector<int> queue{ from };
			for (size_t head = 0; head < queue.size(); head++)
			{
				auto k = queue[head];
				auto cx = k % XRES, cy = k / XRES;
				for (auto [dx, dy] : { std::pair(0, -1), { -1, 0 }, { 1, 0 }, { 0, 1 } })
				{
					auto nx = cx + dx, ny = cy + dy;
					auto nk = ny * XRES + nx;
					if (outside(nx, ny))
					{
						continue;
					}
					auto q = sim.pmap[ny][nx];
					if (q && partBody[ID(q)] == owner)
					{
						if (visited[nk] != visitStamp)
						{
							visited[nk] = visitStamp;
							queue.push_back(nk);
						}
					}
					else if (done(k, nk))
					{
						return std::pair(k, nk);
					}
				}
			}
			return std::pair(-1, -1);
		};
		auto at = y * XRES + x;
		if (on)
		{
			at = search(at, [&](int, int k) {
				return flows(k % XRES, k / XRES) && sideOf(k) == on;
			}).first;
		}
		// Failing a free cell (a body sunk in liquid), it takes the place of
		// liquid off that side, which is then pushed aside in turn.
		auto offSide = [&](int k, bool free) {
			auto x = k % XRES, y = k / XRES;
			return (free ? !sim.pmap[y][x] : flows(x, y)) && !sim.IsWallBlocking(x, y, PT_RIGD) && (!on || sideOf(k) != on);
		};
		auto to = at < 0 ? -1 : search(at, [&](int, int k) { return offSide(k, true); }).second;
		if (at >= 0 && to < 0)
		{
			to = search(at, [&](int, int k) { return offSide(k, false); }).second;
		}
		if (to < 0)
		{
			return false;
		}
		auto displaced = sim.pmap[to / XRES][to % XRES];
		auto q = sim.pmap[at / XRES][at % XRES];
		auto &body = bodies.at(owner);
		auto &p = body.pixels[partIndex[ID(q)]];
		p.wx = to % XRES;
		p.wy = to / XRES;
		parts[ID(q)].x = float(p.wx);
		parts[ID(q)].y = float(p.wy);
		sim.pmap[p.wy][p.wx] = q;
		body.displaced++;
		put(id, at);
		if (displaced)
		{
			auto beyond = sideOf(to);
			if (!findPlaces({ ID(displaced) }, anywhere, beyond).empty() && !findPlaces({ ID(displaced) }, anywhere, 0).empty())
			{
				timings.fluidLost++;
				sim.kill_part(ID(displaced));
			}
		}
		return true;
	};
	for (size_t m = 0; m < moves.size(); m++)
	{
		if (pushed[m].empty())
		{
			continue;
		}
		findSides(m);
		// The side each pushed particle was on: that of the nearest cell that
		// liquid can be in, out through the body (it's moved over the
		// particle, so the side it came in from is nearest), preferably one
		// the body didn't just leave (across a thin wall, that can be the
		// other side), or 0 if there's none near.
		visitStamp++;
		for (auto k : left[m])
		{
			visited[k] = visitStamp;
		}
		auto sideNear = [&](int id) {
			auto x = int(parts[id].x + 0.5f), y = int(parts[id].y + 0.5f);
			std::vector<int> layer{ y * XRES + x }, next, seen = layer;
			for (int d = 0; d < maxSideSearch && !layer.empty(); d++)
			{
				int vacated = 0;
				next.clear();
				for (auto k : layer)
				{
					for (auto [dx, dy] : { std::pair(0, -1), { -1, 0 }, { 1, 0 }, { 0, 1 } })
					{
						auto nx = k % XRES + dx, ny = k / XRES + dy;
						auto nk = ny * XRES + nx;
						if (!inBox(nx, ny) || std::find(seen.begin(), seen.end(), nk) != seen.end())
						{
							continue;
						}
						seen.push_back(nk);
						if (!flows(nx, ny))
						{
							next.push_back(nk);
						}
						else if (visited[nk] != visitStamp)
						{
							return sideOf(nk);
						}
						else
						{
							vacated = sideOf(nk);
						}
					}
				}
				if (vacated)
				{
					return vacated;
				}
				std::swap(layer, next);
			}
			return 0;
		};
		std::vector<std::pair<int, int>> fluid; // side, particle ID
		for (auto id : pushed[m])
		{
			fluid.emplace_back(sideNear(id), id);
		}
		auto v = b2Body_GetLinearVelocity(moves[m].body->b2);
		auto across = b2Length(v) > 1e-3f ? b2Vec2{ -v.y, v.x } : b2Vec2{ 1.0f, 0.0f };
		auto &cells = left[m];
		cells.erase(std::remove_if(cells.begin(), cells.end(), [&](int k) {
			auto x = k % XRES, y = k / XRES;
			return sim.pmap[y][x] || !(liquidAt(x - 1, y) || liquidAt(x + 1, y) || liquidAt(x, y - 1) || liquidAt(x, y + 1));
		}), cells.end());
		std::sort(cells.begin(), cells.end(), [&](int a, int b) {
			auto sa = sideOf(a), sb = sideOf(b);
			if (sa != sb)
			{
				return sa < sb;
			}
			return across.x * (a % XRES) + across.y * (a / XRES) < across.x * (b % XRES) + across.y * (b / XRES);
		});
		std::sort(fluid.begin(), fluid.end(), [&](auto &a, auto &b) {
			if (a.first != b.first)
			{
				return a.first < b.first;
			}
			return across.x * parts[a.second].x + across.y * parts[a.second].y < across.x * parts[b.second].x + across.y * parts[b.second].y;
		});
		// Each side's liquid goes into the cells the body left on that side.
		std::vector<std::pair<int, int>> unplaced;
		size_t ci = 0;
		for (size_t i = 0; i < fluid.size(); )
		{
			auto on = fluid[i].first;
			auto j = i;
			while (j < fluid.size() && fluid[j].first == on)
			{
				j++;
			}
			while (ci < cells.size() && sideOf(cells[ci]) < on)
			{
				ci++;
			}
			auto cj = ci;
			while (cj < cells.size() && sideOf(cells[cj]) == on)
			{
				cj++;
			}
			auto d = j - i, c = on ? cj - ci : 0;
			for (size_t n = 0; n < d; n++)
			{
				if (d <= c || n * c / d != (n + 1) * c / d)
				{
					put(fluid[i + n].second, cells[ci + n * c / d]);
				}
				else
				{
					unplaced.push_back(fluid[i + n]);
				}
			}
			i = j;
			ci = cj;
		}
		// Liquid of different kinds goes separately, each through its own
		// kind, and so does liquid on different sides.
		std::stable_sort(unplaced.begin(), unplaced.end(), [&](auto &a, auto &b) {
			auto ta = parts[a.second].type, tb = parts[b.second].type;
			return ta != tb ? ta < tb : a.first < b.first;
		});
		for (size_t i = 0; i < unplaced.size(); )
		{
			auto type = parts[unplaced[i].second].type;
			auto on = unplaced[i].first;
			std::vector<int> group;
			auto j = i;
			while (j < unplaced.size() && parts[unplaced[j].second].type == type && unplaced[j].first == on)
			{
				group.push_back(unplaced[j++].second);
			}
			auto same = [&](int x, int y) {
				return TYP(sim.pmap[y][x]) == type;
			};
			auto left = findPlaces(group, same, on);
			left = findPlaces(left, notRigid, on);
			// Liquid caught in a notch that the body closed over has no way
			// out but through the body, to its own side (a hollow is all
			// within the margin round the body).
			if (on > 1)
			{
				left = findPlaces(left, inBox, on);
			}
			else
			{
				left = findPlaces(left, anywhere, on);
			}
			// Failing that, rather than lose it, it goes anywhere free at
			// all; only with the whole screen full is anything removed.
			for (auto id : left)
			{
				if (!makeRoom(id, on) && !findPlaces({ id }, anywhere, 0).empty())
				{
					timings.fluidLost++;
					sim.kill_part(id);
				}
			}
			i = j;
		}
	}
}
