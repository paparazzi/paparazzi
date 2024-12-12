/*
 * Copyright (C) 2024-2025 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include <boost/iostreams/device/mapped_file.hpp> // for mmap
#include <boost/filesystem.hpp> // for is_empty
#include <algorithm>  // for std::find
#include <iostream>   // for std::cout
#include <cstring>

#include <pprzlink/MessageDictionary.h>

bool parse_line(std::string line) {
    return true;
}


int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <filename>\n";
        return 1;
    }


    if (boost::filesystem::is_empty(argv[1])) {
        std::cout << " - Empty DATA file\n";
        return 0;
    }

    std::cout << "LOG: " << argv[1] << "\n";


    boost::iostreams::mapped_file mmap(argv[1], boost::iostreams::mapped_file::readonly);
    auto f = mmap.const_data();
    auto l = f + mmap.size();


    // replace the .data in argv[1] with .log
    std::string log_file = argv[1];
    log_file.replace(log_file.end()-4, log_file.end(), "log");

    std::cout << " - log_file = " << log_file << "\n";


    if (boost::filesystem::is_empty(log_file)) {
        std::cout << " - Empty LOG file\n";
        return 0;
    }

    // Load message definitions from *.LOG / messages.xml
    pprzlink::MessageDictionary *dict = new pprzlink::MessageDictionary(log_file);
//    pprzlink::MessageDictionary *dict = new pprzlink::MessageDictionary("./pprzlink/message_definitions/v1.0/messages.xml");


    const char *PIC = "[PFC] pic:";

    // Find the first occurrence of the string "[PFC] pic:"
    auto it = std::search(f, l, PIC, PIC + strlen(PIC));
    if (it != l) {
        // Print the line where the string was found
        auto start = it;
        while (start != f && *start != '\n') start--;
        start++;
        auto end = it;
        while (end != l && *end != '\n') end++;
        std::cout << " - Found: " << std::string(start, end) << "\n";
        return 0;
    }

    // Find the number of lines in the file
    uintmax_t m_numLines = 0;
    while (f && f!=l)
        if ((f = static_cast<const char*>(memchr(f, '\n', l-f))))
            m_numLines++, f++;



    std::cout << " - m_numLines = " << m_numLines << "\n";
//    std::cout << " - mmap.size() = " << mmap.size() << "\n";
}
