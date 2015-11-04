#!/bin/bash

if [ $# -ne 2 ]; then
  echo "Usage: import.sh fieldlist classname"
  exit
fi

echo "#ifndef $2_HPP_INC"
echo "#define $2_HPP_INC"
echo "class $2{"
echo "public:"
# Variables
cat $1 |tr ":" " " |sed -e "s/FTString/std::string/g" |sed -e "s/FTInteger/long/g" |sed -e "s/FTDouble/double/g" |sed -e "s/\$/;/g" |sed -e "s/^/\t/g"
# Field names

echo "static inline std::string fieldname(size_t id) {"
echo "switch(id){"

# Read from ID

I=0
while read line; do
echo -ne "\t"
TYPE=$(echo $line | cut -d: -f1)
NAME=$(echo $line | cut -d: -f2)
echo "case $I: return std::string(\"$NAME\");";
I=$(echo $I+1 |bc)
done < $1
echo -ne "\t"
echo "default: return std::string(\"unknown fieldname\");"
echo "}}; //fieldname"

# Format checks
cat << "EOF"
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
EOF


echo "void readFromID(DBFHandle hDBF, size_t id) {"

# Read from ID

I=0
while read line; do
echo -ne "\t"
TYPE=$(echo $line | cut -d: -f1)
NAME=$(echo $line | cut -d: -f2)
case $TYPE in 
  "FTString")
	echo "$NAME = std::string (DBFReadStringAttribute(hDBF,id, $I));"
	;;

  "FTInteger")
	echo "$NAME = DBFReadIntegerAttribute(hDBF,id, $I);"
	;;
   "FTDouble")
   	echo "$NAME = _DBFReadDoubleAttribute(hDBF,id, $I);"
	;;
	*)
		echo not implementede $TYPE--$NAME
		exit 
		;;

esac
I=$(echo $I+1 |bc)
done < $1
echo "};"

cat << EOF
void dump()
{
	dump(cout);
}
EOF

# Dump
echo "template<typename stream> void dump(stream &o) {"
I=0
while read line; do
echo -ne "\t"
TYPE=$(echo $line | cut -d: -f1)
NAME=$(echo $line | cut -d: -f2)
    echo "o << \"$NAME\"  <<\":\\t\" <<  $NAME << std::endl;";
done < $1
echo "};"



echo "};"
echo "#endif"
