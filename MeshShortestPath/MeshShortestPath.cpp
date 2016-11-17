// MeshShortestPath.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <list>
#include <algorithm>
#include <vector>
#include <cassert>
#include <memory>

#include "Types.h"
#include "GenerateGeometry.h"

namespace MeshShortestPath {

	static void printPoint(std::ostream& stream, Polyhedron::Point point) {
		stream << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")";
	}

	template <typename T>
	static void iterateHalfedges(Polyhedron::Facet_handle facet, T callback) {
		auto halfedge = facet->halfedge();
		auto halfedge2 = halfedge;
		do {
			halfedge2 = halfedge2->next();
			callback(halfedge2);
		} while (halfedge2 != halfedge);
	}

	static Polyhedron::Point middlePoint(const Polyhedron::Facet_handle facet) {
		std::vector<Polyhedron::Point> points;
		iterateHalfedges(facet, [&](Polyhedron::Halfedge_handle halfedge) {
			points.emplace_back(halfedge->vertex()->point());
		});
		Polyhedron::Point result(0, 0, 0);
		for (auto point : points) {
			result = Polyhedron::Point(result.x() + point.x(), result.y() + point.y(), result.z() + point.z());
		}
		return Polyhedron::Point(result.x() / points.size(), result.y() / points.size(), result.z() / points.size());
	}

	class MMP {
	public:
		// FIXME: The input can either be a point on the interior of a triangle, a point on the interior of an edge, or a vertex.
		MMP(Polyhedron& polyhedron, Polyhedron::Facet_handle initialFacet, Polyhedron::Point pointOnInitialFacet) : polyhedron(polyhedron) {
			std::vector<Polyhedron::Point> initialFacetPoints;
			iterateHalfedges(initialFacet, [&](Polyhedron::Halfedge_handle halfedge) {
				initialFacetPoints.push_back(halfedge->vertex()->point());
			});
			std::cout << "Starting with point ";
			printPoint(std::cout, pointOnInitialFacet);
			std::cout << " on facet ";
			for (auto& point : initialFacetPoints) {
				if (&point != &initialFacetPoints.front())
					std::cout << ", ";
				printPoint(std::cout, point);
			}
			std::cout << std::endl;

			iterateHalfedges(initialFacet, [&](Polyhedron::Halfedge_handle halfedge) {
				CandidateInterval interval(halfedge, pointOnInitialFacet, pointOnInitialFacet, 0, 0, 1);
				candidateIntervals.emplace_back(std::move(interval));
				std::list<CandidateInterval>::iterator iterator = candidateIntervals.end();
				--iterator;
				assert(&*iterator == &candidateIntervals.back());
				bool firstOrLastOnHalfedge = halfedge->insertInterval(iterator);
				iterator->setFrontierPointIsAtVertex(iterator->getFrontierPointIsAtExtent() && firstOrLastOnHalfedge);
				eventQueue.place(FrontierPointEvent(iterator->getFrontierPoint(), iterator));
				eventQueue.place(EndPointEvent(halfedge->vertex()->point(), *iterator));
			});
		}

		void populate() {
			while (!eventQueue.empty()) {
				GenericEvent e = eventQueue.remove();
				boost::apply_visitor(EventVisitor(*this), e);
			}
		}

		~MMP() {
			// FIXME: Find some way to export the data
			for (auto i = polyhedron.halfedges_begin(); i != polyhedron.halfedges_end(); ++i) {
				i->clear();
			}
		}

	private:
		class EventVisitor {
		public:
			EventVisitor(MMP& mmp) : mmp(mmp) {
			}

			void operator()(const FrontierPointEvent& event) const {
				std::cout << "Encountering event with label " << event.getLabel() << std::endl;
				// FIXME: Possibly label the event
				mmp.propagate(*event.getCandidateInterval());
			}

			void operator()(const EndPointEvent& event) const {
				std::cout << "Encountering event with label " << event.getLabel() << std::endl;
				// FIXME: Possibly label the event
			}

		private:
			MMP& mmp;
		};

		void propagate(const CandidateInterval& interval) {
			if (interval.getFrontierPointIsAtVertex()) {
				// FIXME: Find the vertex, then find opposite edges which are greater than 2*pi, then insertInteval()
			}
			else {
				// FIXME: do project() and insertInterval()
			}
		}

		Polyhedron& polyhedron;
		std::list<CandidateInterval> candidateIntervals;
		EventQueue eventQueue;
	};

}

int main()
{
	MeshShortestPath::Polyhedron polyhedron = MeshShortestPath::generateCube();
	auto facet = polyhedron.facets_begin();
	MeshShortestPath::MMP mmp(polyhedron, facet, middlePoint(facet));
	mmp.populate();
    return 0;
}

