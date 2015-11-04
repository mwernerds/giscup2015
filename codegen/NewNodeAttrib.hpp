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
 */#ifndef NewNodeAttrib_HPP_INC
#define NewNodeAttrib_HPP_INC
class NewNodeAttrib{
public:
	double ID;
static inline std::string fieldname(size_t id) {
switch(id){
	case 0: return std::string("ID");
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
	ID = _DBFReadDoubleAttribute(hDBF,id, 0);
};
void dump()
{
	dump(cout);
}
template<typename stream> void dump(stream &o) {
	o << "ID"  <<":\t" <<  ID << std::endl;
};
};
#endif
