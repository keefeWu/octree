#ifndef _STRINGUTILS_HPP_
#define _STRINGUTILS_HPP_

#include <string>
#include <sstream>
#include <vector>

#include <algorithm>

#include "alphanum.hpp"

namespace strutils {

/*
 * Replace a keyword in a string with another word
 * e.g. replaceStr("Hello world!!!", "world", "john")
 * returns "Hello john!!!"
 */
inline std::string replaceStr(std::string str, std::string keyword,
		std::string replacement) {
	size_t index = 0;
	std::string str2;
	while (true) {
		/* Locate the substring to replace. */
		index = str.find(keyword, index);
		if (index == std::string::npos)
			break;

		/* Make the replacement. */
		unsigned int wlen = replacement.length();
		if (wlen + index + 1 < str.length())
			str2 = str.replace(index, wlen-1, replacement);
		else
			str2 = str.replace(index, wlen, replacement);

		/* Advance index forward so the next iteration doesn't pick it up as well. */
		index += wlen;
	}
	return str2;
}

inline std::vector<std::string> splitStr(std::string str, char delimiter) {
	std::vector<std::string> internal;
	std::stringstream ss(str); // Turn the string into a stream.
	std::string tok;

	while (std::getline(ss, tok, delimiter)) {
		internal.push_back(tok);
	}
	return internal;
}

template < class ContainerT >
inline void tokenize(ContainerT& tokens, const std::string str,
              const std::string delimiters = " ")
{
	tokens.clear();
    std::string::size_type pos, lastPos = 0, length = str.length();

    using value_type = typename ContainerT::value_type;
    using size_type  = typename ContainerT::size_type;

    while(lastPos < length + 1)
    {
		pos = str.find(delimiters, lastPos);
		if(pos == std::string::npos)
		{
			pos = length;
		}

		if(pos != lastPos)
		{
	 		tokens.push_back(value_type(str.data()+lastPos,
		       (size_type)pos-lastPos ));
		}

		lastPos = pos + delimiters.length();
    }
}


inline void sort_strings_numerically(std::vector<std::string>& strings_)
{
	std::sort(strings_.begin(), strings_.end(), doj::alphanum_less<std::string>());
}


} /*namespace strutils*/

#endif
