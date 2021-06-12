RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color
printf "${GREEN}DODO ALIVE PROJECT!${NC}\n"
printf "${GREEN}CMAKE${NC}"
rm -rf build
mkdir build
cd build
cmake ..
cmake --build .
printf "${GREEN}Start calculations${NC}\n"
./dynamics $1 $2
printf "${GREEN}Start Meshup viso${NC}\n"
meshup ../model/articulatedLeg.lua ../output/animationArticulatedLegModel.csv
printf "${GREEN} END${NC}"
