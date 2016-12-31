#include "pch.h"
#include "MMP.h"

#define CGAL_CHECK_EXPENSIVE
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#undef CGAL_CHECK_EXPENSIVE

#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include <cassert>
#include <algorithm>
#include <list>
#include <sstream>

typedef CGAL::Simple_cartesian<double> Kernel;

class CandidateInterval;

template <class Refs>
class MMPHalfedge : public CGAL::HalfedgeDS_halfedge_base<Refs> {
public:
	uint8_t getIndex() const {
		return index;
	}

	void setIndex(uint8_t index) {
		this->index = index;
	}

	void insertInitialInterval(std::list<CandidateInterval>::iterator interval) {
		assert(intervals.empty());
		intervals.push_back(interval);
	}

	void iterateIntervals(std::function<void(const CandidateInterval&)> iterator) const {
		for (auto interval : intervals) {
			iterator(*interval);
		}
	}

private:
	uint8_t index; // 0 means "pointing from vertex 0 to vertex 1"
	std::vector<std::list<CandidateInterval>::iterator> intervals;
};

template <class Refs, class Traits>
class MMPFace : public CGAL::HalfedgeDS_face_base<Refs, CGAL::Tag_true, typename Traits::Plane_3> {
public:
	MMP::TriangleIndices::size_type getIndex() const {
		return index;
	}

	void setIndex(MMP::TriangleIndices::size_type index) {
		this->index = index;
	}

private:
	MMP::TriangleIndices::size_type index;
};

struct MMP_Items : public CGAL::Polyhedron_items_3 {
	template <class Refs, class Traits>
	struct Halfedge_wrapper {
		typedef MMPHalfedge<Refs> Halfedge;
	};
	template <class Refs, class Traits>
	struct Face_wrapper {
		typedef typename Traits::Plane_3 Plane;
		typedef MMPFace<Refs, Traits> Face;
	};
};

typedef CGAL::Polyhedron_3<Kernel, MMP_Items> Polyhedron;

class CandidateInterval {
public:
	CandidateInterval(Polyhedron::Halfedge_handle halfedge, Kernel::Point_3 unfoldedRoot, Kernel::FT depth, Kernel::FT lowerExtent, Kernel::FT upperExtent) : halfedge(halfedge), unfoldedRoot(unfoldedRoot), depth(depth), lowerExtent(lowerExtent), upperExtent(upperExtent) {
	}

	Polyhedron::Point getUnfoldedRoot() const {
		return unfoldedRoot;
	}

	Kernel::FT getDepth() const {
		return depth;
	}

	Kernel::FT getUpperExtentFraction() const {
		return upperExtent;
	}

	Kernel::Point_3 getFrontierPoint() const {
		return calculatePoint(frontierPoint);
	}

	Kernel::Point_3 getLowerExtent() const {
		return calculatePoint(lowerExtent);
	}

	Kernel::Point_3 getUpperExtent() const {
		return calculatePoint(upperExtent);
	}

private:
	Kernel::Point_3 calculatePoint(Kernel::FT fraction) const {
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		auto s = destination - source;
		return source + fraction * s;
	}

	Polyhedron::Point unfoldedRoot;
	Polyhedron::Halfedge_handle halfedge;
	Kernel::FT depth;
	Kernel::FT lowerExtent; // 0 <= lowerExtent <= frontierPoint <= upperExtent <= 1
	Kernel::FT upperExtent;
	Kernel::FT frontierPoint;
};

class MMP::Impl {
public:
	// FIXME: This is too restrictive.
	Impl() = delete;
	Impl(const Impl&) = delete;
	Impl(Impl&&) = delete;
	void operator=(const Impl&) = delete;
	void operator=(Impl&&) = delete;

	Impl(PointHeap pointHeap, TriangleIndices triangles, TriangleIndices::size_type startingPointOwningTriangle, double u, double v) {
		// FIXME: Make sure that everything is in bounds.
		Generator generator(pointHeap, triangles, startingPointOwningTriangle);
		polyhedron.delegate(generator);
		// FIXME: Make sure we are a manifold.
		startingFacet = generator.getStartingFacet();

		auto startingTriangle = triangles[startingPointOwningTriangle];
		auto p0 = convertPoint(pointHeap[std::get<0>(startingTriangle)]);
		auto p1 = convertPoint(pointHeap[std::get<1>(startingTriangle)]);
		auto p2 = convertPoint(pointHeap[std::get<2>(startingTriangle)]);
		auto v0 = p1 - p0;
		auto v1 = p2 - p0;
		startingPoint = p0 + u * v0 + v * v1;
	}

	void run() {
		auto halfedge = startingFacet->halfedge();
		do {
			CandidateInterval candidateInterval(halfedge, startingPoint, 0, 0, 1);
			auto iterator = candidateIntervals.insert(candidateIntervals.end(), candidateInterval);
			halfedge->insertInitialInterval(iterator);
			eventQueue.place(FrontierPointEvent(iterator));
			eventQueue.place(EndPointEvent(halfedge->vertex()->point(), *iterator));
			halfedge = halfedge->next();
		} while (halfedge != startingFacet->halfedge());

		while (!eventQueue.empty()) {
			GenericEvent e = eventQueue.remove();
			boost::apply_visitor(EventVisitor(*this), e);
		}
	}

	std::vector<std::array<std::vector<HalfedgeInterval>, 3>> intervals() const {
		std::vector<std::array<std::vector<HalfedgeInterval>, 3>> result(polyhedron.size_of_facets());
		for (auto facet = polyhedron.facets_begin(); facet != polyhedron.facets_end(); ++facet) {
			std::array<std::vector<HalfedgeInterval>, 3> edges;
			auto halfedge = facet->halfedge();
			do {
				std::vector<HalfedgeInterval> intervals;
				boost::optional<Kernel::FT> maximumExtent;
				halfedge->iterateIntervals([&](const CandidateInterval& interval) {
					auto unfoldedRoot = interval.getUnfoldedRoot();
					auto upperExtent = interval.getUpperExtentFraction();
					maximumExtent = maximumExtent ? std::max(maximumExtent.get(), upperExtent) : upperExtent;
					intervals.push_back({ { unfoldedRoot.x(), unfoldedRoot.y(), unfoldedRoot.z() }, upperExtent, interval.getDepth() });
				});
				assert(!maximumExtent || maximumExtent.get() == 1);
				edges[halfedge->getIndex()] = std::move(intervals);
				halfedge = halfedge->next();
			} while (halfedge != facet->halfedge());
			result[facet->getIndex()] = std::move(edges);
		}
		return result;
	}

private:
	class Generator : public CGAL::Modifier_base<Polyhedron::HalfedgeDS> {
	public:
		Generator(const MMP::PointHeap& pointHeap, const MMP::TriangleIndices& triangles, MMP::TriangleIndices::size_type startingPointOwningTriangle) : pointHeap(pointHeap), triangles(triangles), startingPointOwningTriangle(startingPointOwningTriangle) {
		}

		Polyhedron::Facet_handle getStartingFacet() {
			return startingFacet;
		}

	private:
		void operator()(Polyhedron::HalfedgeDS& halfedgeDS) {
			CGAL::Polyhedron_incremental_builder_3<Polyhedron::HalfedgeDS> builder(halfedgeDS, true);
			builder.begin_surface(pointHeap.size(), triangles.size());
			std::vector<Polyhedron::Vertex_handle> vertices;
			for (MMP::PointHeap::size_type i = 0; i < pointHeap.size(); ++i) {
				const auto& point = pointHeap[i];
				auto vertex = builder.add_vertex(convertPoint(point));
				vertices.push_back(vertex);
			}
			for (MMP::TriangleIndices::size_type i = 0; i < triangles.size(); ++i) {
				const auto& triangle = triangles[i];
				auto facet = builder.begin_facet();
				builder.add_vertex_to_facet(std::get<0>(triangle));
				builder.add_vertex_to_facet(std::get<1>(triangle));
				builder.add_vertex_to_facet(std::get<2>(triangle));
				auto halfedge = builder.end_facet();

				assert(halfedge->vertex() == vertices[std::get<0>(triangle)]);
				assert(halfedge->next()->vertex() == vertices[std::get<1>(triangle)]);
				assert(halfedge->next()->next()->vertex() == vertices[std::get<2>(triangle)]);
				assert(halfedge->next()->next()->next() == halfedge);

				facet->setIndex(i);
				auto p0 = halfedge->vertex()->point();
				auto p1 = halfedge->next()->vertex()->point();
				auto p2 = halfedge->next()->next()->vertex()->point();
				facet->plane() = Polyhedron::Plane_3(p0, p1, p2);
				halfedge->setIndex(2);
				halfedge->next()->setIndex(0);
				halfedge->next()->next()->setIndex(1);
				if (i == startingPointOwningTriangle) {
					startingFacet = facet;
				}
			}
			builder.end_surface();
		}

		const MMP::PointHeap& pointHeap;
		const MMP::TriangleIndices& triangles;
		MMP::TriangleIndices::size_type startingPointOwningTriangle;
		Polyhedron::Facet_handle startingFacet;
	};

	class FrontierPointEvent {
	public:
		FrontierPointEvent(std::list<CandidateInterval>::iterator candidateInterval) : point(candidateInterval->getFrontierPoint()), candidateInterval(candidateInterval) {
		}

		std::list<CandidateInterval>::iterator getCandidateInterval() const {
			return candidateInterval;
		}

		Kernel::FT FrontierPointEvent::getLabel() const {
			return computeLabel(point, *candidateInterval);
		}

	private:
		Kernel::Point_3 point;
		std::list<CandidateInterval>::iterator candidateInterval;
	};

	class EndPointEvent {
	public:
		EndPointEvent(Kernel::Point_3 point, const CandidateInterval& candidateInterval) : point(point), label(computeLabel(point, candidateInterval)) {
		}

		Kernel::FT getLabel() const {
			return label;
		}

	private:
		Kernel::Point_3 point;
		Kernel::FT label;
	};

	typedef boost::variant<FrontierPointEvent, EndPointEvent> GenericEvent;

	class EventQueue {
	public:
		void EventQueue::place(FrontierPointEvent& event) {
			heap.push_back(event);
			std::push_heap(heap.begin(), heap.end(), &eventComparison);
		}

		void EventQueue::place(EndPointEvent& event) {
			heap.push_back(event);
			std::push_heap(heap.begin(), heap.end(), &eventComparison);
		}

		GenericEvent EventQueue::remove() {
			std::pop_heap(heap.begin(), heap.end(), &eventComparison);
			GenericEvent result = heap.back();
			heap.pop_back();
			return result;
		}

		bool empty() const {
			return heap.empty();
		}

	private:
		class EventComparison {
		public:
			Kernel::FT operator()(const FrontierPointEvent& event) const {
				return event.getLabel();
			}

			Kernel::FT operator()(const EndPointEvent& event) const {
				return event.getLabel();
			}
		};

		static bool eventComparison(const GenericEvent& a, const GenericEvent& b) {
			Kernel::FT aLabel = boost::apply_visitor(EventComparison(), a);
			Kernel::FT bLabel = boost::apply_visitor(EventComparison(), b);
			return aLabel > bLabel;
		}

		std::vector<GenericEvent> heap;
	};

	class EventVisitor {
	public:
		EventVisitor(Impl& impl) : impl(impl) {
		}

		void operator()(const FrontierPointEvent& event) const {
			/*if (event.getCandidateInterval()->isDeleted()) {
				mmp.candidateIntervals.erase(event.getCandidateInterval());
				return;
			}*/

			// FIXME: Possibly label the event
			impl.propagate(*event.getCandidateInterval());
		}

		void operator()(const EndPointEvent& event) const {
			// FIXME: Possibly label the event
		}

	private:
		Impl& impl;
	};

	static Polyhedron::Point convertPoint(const MMP::PointHeap::value_type& point) {
		return Polyhedron::Point(std::get<0>(point), std::get<1>(point), std::get<2>(point));
	};

	static Kernel::FT computeLabel(Kernel::Point_3 point, const CandidateInterval& candidateInterval) {
		auto unfoldedRoot = candidateInterval.getUnfoldedRoot();
		auto displacement = point - unfoldedRoot;
		return std::sqrt(displacement.squared_length()) + candidateInterval.getDepth();
	}

	void propagate(const CandidateInterval& interval) {
		// FIXME: Implement this.
	}

	Polyhedron polyhedron;
	std::list<CandidateInterval> candidateIntervals;
	EventQueue eventQueue;
	Polyhedron::Facet_handle startingFacet;
	Polyhedron::Point startingPoint;
};

MMP::MMP(PointHeap pointHeap, TriangleIndices triangles, TriangleIndices::size_type startingPointOwningTriangle, double u, double v) : impl(std::make_unique<Impl>(pointHeap, triangles, startingPointOwningTriangle, u, v)) {
}

MMP::~MMP() {
}

void MMP::run() {
	impl->run();
}

auto MMP::intervals() const -> std::vector<std::array<std::vector<HalfedgeInterval>, 3>> {
	return impl->intervals();
}
