#include <boost/iostreams/device/mapped_file.hpp> // for mmap
#include <boost/filesystem.hpp> // for is_empty
#include <boost/regex.hpp>
#include <boost/spirit/home/x3.hpp>
#include <boost/spirit/include/qi_lazy.hpp>
#include <algorithm>  // for std::find
#include <iostream>   // for std::cout
#include <cstring>
#include <tinyxml2.h>
#include <iostream>
#include <regex>

#include <pprzlink/MessageDictionary.h>
#include <pprzlink/MessageDefinition.h>
#include <pprzlink/Message.h>
#include <pprzlink/MessageFieldTypes.h>

using namespace boost::spirit::x3;


void parse_airframe_list(tinyxml2::XMLElement *root)
{
  auto aircraft = root->FirstChildElement("conf")->FirstChildElement("aircraft");
  while (aircraft != nullptr) {
    auto className = aircraft->Attribute("name", nullptr);
    if (className == nullptr) {
      className = aircraft->Attribute("NAME", nullptr);
    }
    int classId = aircraft->IntAttribute("ac_id", -1);
    if (classId == -1) {
      classId = aircraft->IntAttribute("AC_ID", -1);
    }
    if (className == nullptr || classId == -1) {
      std::cout << "aircraft has no name or ac_id.";
    }
    // std::cout << " - aircraft: " << className << " id: " << classId << "\n";
    aircraft = aircraft->NextSiblingElement("aircraft");
  }
}

/* Generate a pprzlink message */
pprzlink::Message get_msg(std::string name, pprzlink::MessageDictionary *dict,
                          std::vector<std::vector<boost::variant<int, double, std::string>>> values)
{
  pprzlink::MessageDefinition def = dict->getDefinition(name);
  pprzlink::Message msg(def);
  for (size_t i = 0; i < def.getNbFields(); i++) {
    auto field = def.getField(i);

    switch (field.getType().getBaseType()) {
      case pprzlink::BaseType::INT8:
      case pprzlink::BaseType::INT16:
      case pprzlink::BaseType::INT32:
      case pprzlink::BaseType::UINT8:
      case pprzlink::BaseType::UINT16:
      case pprzlink::BaseType::UINT32:
        if (field.getType().isArray()) {
          std::vector<int> vals;
          for (auto val : values[i]) {
            vals.push_back(boost::get<int>(val));
          }
          msg.addField(field.getName(), vals);
        } else {
          msg.addField(field.getName(), boost::get<int>(values[i][0]));
        }
        break;
      case pprzlink::BaseType::FLOAT:
      case pprzlink::BaseType::DOUBLE:
        if (field.getType().isArray()) {
          std::vector<double> vals;
          for (auto val : values[i]) {
            vals.push_back(boost::get<double>(val));
          }
          msg.addField(field.getName(), vals);
        } else {
          msg.addField(field.getName(), boost::get<double>(values[i][0]));
        }
        break;
      case pprzlink::BaseType::STRING:
        msg.addField(field.getName(), boost::get<std::string>(values[i][0]));
        break;
      case pprzlink::BaseType::CHAR:
        if (field.getType().isArray()) {
          std::string vals;
          size_t end = i + 1;
          if (i + 1 == def.getNbFields()) {
            end = values.size();
          }

          for (size_t j = i; j < end; j++) {
            for (size_t k = 0; k < values[j].size(); k++) {
              auto val = boost::get<std::string>(values[j][k]);
              vals += val;
              vals += ",";
            }
            vals.resize(vals.length() - 1);
            vals += " ";
          }
          vals.resize(vals.length() - 1);

          if (vals[0] == '"') {
            vals = vals.substr(1, vals.size() - 2);
          }
          msg.addField(field.getName(), vals);

        } else {
          msg.addField(field.getName(), boost::get<std::string>(values[i][0]));
        }
        break;
      default:
        break;
    }
  }
  return msg;
}



int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <filename>\n";
    return 1;
  }

  // Check if the data file exists
  if (boost::filesystem::is_empty(argv[1])) {
    std::cerr << " - Empty DATA file\n";
    return 1;
  }
  std::cout << "LOG: " << argv[1] << "\n";

  // Replace and check if the log file exists
  std::string log_file = argv[1];
  log_file.replace(log_file.end() - 4, log_file.end(), "log");
  if (boost::filesystem::is_empty(log_file)) {
    std::cerr << " - Empty LOG file\n";
    return 1;
  }

  // Open the .log file in the xml parser
  tinyxml2::XMLDocument xml;
  xml.LoadFile(log_file.c_str());

  // Check if the root element is configuration
  tinyxml2::XMLElement *root = xml.RootElement();
  std::string rootElem(root->Value());
  if (rootElem != "configuration") {
    std::cerr << "Root element is not configuration in xml messages file (found " + rootElem + ").";
    return 1;
  }

  // Load message definitions from *.LOG / messages.xml
  pprzlink::MessageDictionary *dict = new pprzlink::MessageDictionary(root->FirstChildElement("protocol"));

  // Load all aircraft names
  parse_airframe_list(root);



  // GPS_INT message
  auto gps_int = [&](auto & ctx) {
    // auto timestamp = boost::fusion::at_c<0>(_attr(ctx));
    // auto ac_id = uint8_t(boost::fusion::at_c<1>(_attr(ctx)));
    // auto values = boost::fusion::at_c<2>(_attr(ctx));
    // auto msg = get_msg("GPS_INT", dict, values);
    // msg.setSenderId(ac_id);

    // std::cout << " - GPS_INT: " << timestamp << " " << msg.toString() << "\n";
  };

  // INFO_MSG message
  auto info_msg = [&](auto & ctx) {
    auto timestamp = boost::fusion::at_c<0>(_attr(ctx));
    auto ac_id = uint8_t(boost::fusion::at_c<1>(_attr(ctx)));
    auto values = boost::fusion::at_c<2>(_attr(ctx));
    auto msg = get_msg("INFO_MSG", dict, values);
    msg.setSenderId(ac_id);

    std::cout << " - INFO_MSG: " << timestamp << " " << msg.toString() << "\n";
  };

  // Add the parser
  auto var_types = int_ | double_ | lexeme[+~char_("\r\n")];
  auto var_options = (var_types % ',');
  auto matcher = (float_ >> ' ' >> int_ >> ' ' >> "GPS_INT" >> ' ' >> (var_options % ' ') >> eol)[gps_int] \
                 | (float_ >> ' ' >> int_ >> ' ' >> "INFO_MSG" >> ' ' >> (var_options % ' ') >> eol)[info_msg];
  auto res = matcher | ( * ~char_("\r\n") >> eol);

  // Parse the file
  boost::iostreams::mapped_file_source file(argv[1]);
  parse(file.begin(), file.end(), *(res));
}
