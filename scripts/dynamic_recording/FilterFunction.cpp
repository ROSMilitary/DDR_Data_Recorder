// <!--******************************************************************************
// #
// #"Distribution A: Approved for public release; distribution unlimited. OPSEC #4046"
// #
// PROJECT: DDR
//
//  PACKAGE         :
//  ORIGINAL AUTHOR :
//  MODIFIED DATE   :
//  MODIFIED BY     :
//  REVISION        :
//
//  Copyright (c) 2020 DCS Corporation
//
//  Unlimited Rights assigned to the U.S. Government
//
//  This material may be reproduced by or for the U.S Government pursuant
//  to the copyright license under the clause at DFARS 252.227-7013.  This
//  notice must appear in all copies of this file and its derivatives.
// ******************************************************************************
//
// Copyright (c) 2019-2020 U.S. Federal Government (in countries where recognized)
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
// Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
// #
// #  Licensed under the Apache License, Version 2.0 (the "License");
// #  you may not use this file except in compliance with the License.
// #  You may obtain a copy of the License at
// #
// #      http://www.apache.org/licenses/LICENSE-2.0
// #
// #  Unless required by applicable law or agreed to in writing, software
// #  distributed under the License is distributed on an "AS IS" BASIS,
// #  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// #  See the License for the specific language governing permissions and
// #  limitations under the License.




#include <stdio.h>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <iostream>

#define foreach BOOST_FOREACH

void FilterFunction(std::string PrevBagName,
                    std::string NewBagName,
                    std::vector<std::string> filterTopicList)
{
   rosbag::Bag PrevBag;
   rosbag::Bag NewBag;

   PrevBag.open(PrevBagName, rosbag::bagmode::Read);
   NewBag.open(NewBagName, rosbag::bagmode::Write);


   rosbag::View view(PrevBag, rosbag::TopicQuery(filterTopicList));

   foreach(rosbag::MessageInstance const msg, view)
   {
      NewBag.write(msg.getTopic(), msg.getTime(), msg);
   }

   PrevBag.close();
   NewBag.close();

   if (remove(PrevBagName.c_str()) != 0) {
      //std::cout << "There was an issue deleting bag: " << PrevBagName << std::endl;
   }
}

int main(int argc, char* argv[])
{
   if (argc < 3) {
      //std::cout << "Not enough arguments." << std::endl;
      // return 0;
      std::cout << 0;
      std::cout << '\n' << argv[0] << ' ' << argv[1] << '\n';
   }


   std::vector<std::string> filterTopicList;
   int counter = 3;
   while (argv[counter]) {
      filterTopicList.push_back(argv[counter]);
      counter++;
   }

   FilterFunction(argv[1], argv[2], filterTopicList);

   std::cout << '1';
   // return 1;
}
