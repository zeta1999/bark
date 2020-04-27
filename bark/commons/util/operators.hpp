// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_COMMONS_UTIL_OPERATORS_HPP_
#define MODULES_COMMONS_UTIL_OPERATORS_HPP_

#include <ostream>
#include <vector>
#include <boost/variant.hpp>

template < class T >
inline std::ostream& operator << (std::ostream& os, const std::vector<T>& v) 
{
    os << "[";
    for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
    {
        os << " ";
        os.operator<<(*ii);
    }
    os << " ]";
    return os;
}


template <>
inline std::ostream& operator << <std::vector<float>>(std::ostream& os, const std::vector<std::vector<float>>& v) 
{
    os << "[";
    for (const auto sb : v)
    {
      os << "[";
      for (const auto e : sb) {
        os << " " << e;
      }
      os << " ]";
    }
    os << " ]";
    return os;
}




#endif // MODULES_COMMONS_UTIL_OPERATORS_HPP_