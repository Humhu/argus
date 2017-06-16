#include <string>
#include <vector>
#include "argus_utils/utils/LinalgTypes.h"

namespace argus
{

MatrixType promote_3d_matrix( bool twoDimensional, unsigned int order );

/*! \brief Parses strings of the format [type]_[dim]_[order] to indices
 * for a filter. Assumes order 0 corresponds to pose, 1 to velocity, etc.
 *
 * Parameter minOrder is subtracted from the specified order to allow
 * offsetting, ie. for a velocity-only filter, minOrder = 1
 */
unsigned int parse_dim_string( const std::string& s,
                               bool twoDimensional,
                               unsigned int maxOrder,
                               unsigned int minOrder = 0 );

/*! \brief Convenience method for parsing containers of strings. */
// TODO Upgrade to a template-template to take different containers...
std::vector<unsigned int> parse_dim_string( const std::vector<std::string>& s,
                                            bool twoDimensional,
                                            unsigned int maxOrder,
                                            unsigned int minOrder = 0 );
}