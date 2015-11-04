/* Copyright 2015 Martin Werner - <martin.werner@ifi.lmu.de>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */#ifndef NewRoadAttrib_HPP_INC
#define NewRoadAttrib_HPP_INC
class NewRoadAttrib{
public:
	long EDGEID;
	long STARTID;
	long ENDID;
	double LENGTH;
	double SPD;
static inline std::string fieldname(size_t id) {
switch(id){
	case 0: return std::string("EDGEID");
	case 1: return std::string("STARTID");
	case 2: return std::string("ENDID");
	case 3: return std::string("LENGTH");
	case 4: return std::string("SPD");
	default: return std::string("unknown fieldname");
}}; //fieldname
static bool assertCorrectFormat(DBFHandle hDBF)
{
	size_t fields = DBFGetFieldCount(hDBF);
        for (size_t i=0; i < fields; i++)
        {
			char name[20];
			size_t type = DBFGetFieldInfo(hDBF,i,name, NULL, NULL);
			if (fieldname(i) != name)
			{
				cout << "File DBF in wrong format" << endl;
				cout << fieldname(i) << " expected. Found " << name << endl;
				return false;
			}
		}
		return true;
}
void readFromID(DBFHandle hDBF, size_t id) {
	EDGEID = DBFReadIntegerAttribute(hDBF,id, 0);
	STARTID = DBFReadIntegerAttribute(hDBF,id, 1);
	ENDID = DBFReadIntegerAttribute(hDBF,id, 2);
	LENGTH = _DBFReadDoubleAttribute(hDBF,id, 3);
	SPD = _DBFReadDoubleAttribute(hDBF,id, 4);
};
void dump()
{
	dump(cout);
}
template<typename stream> void dump(stream &o) {
	o << "EDGEID"  <<":\t" <<  EDGEID << std::endl;
	o << "STARTID"  <<":\t" <<  STARTID << std::endl;
	o << "ENDID"  <<":\t" <<  ENDID << std::endl;
	o << "LENGTH"  <<":\t" <<  LENGTH << std::endl;
	o << "SPD"  <<":\t" <<  SPD << std::endl;
};
};
#endif
