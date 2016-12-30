#include "pch.h"
#include "MMP.h"

#define CGAL_CHECK_EXPENSIVE
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#undef CGAL_CHECK_EXPENSIVE

typedef CGAL::Simple_cartesian<double> Kernel;

template <class Refs>
class HalfedgeWithIntervalVector : public CGAL::HalfedgeDS_halfedge_base<Refs> {
};

struct MMP_Items : public CGAL::Polyhedron_items_3 {
	template <class Refs, class Traits>
	struct Halfedge_wrapper {
		typedef HalfedgeWithIntervalVector<Refs> Halfedge;
	};
};

typedef CGAL::Polyhedron_3<Kernel, MMP_Items> Polyhedron;

class MMP::Impl {
public:
private:
	Polyhedron polyhedron;
};

MMP::MMP(std::vector<std::array<double, 3>> pointHeap, std::vector<std::size_t> indices) : impl(std::make_unique<Impl>()) {

}

MMP::~MMP() {
}
