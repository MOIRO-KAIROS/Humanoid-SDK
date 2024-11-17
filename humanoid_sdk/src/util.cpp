
// int64_t vector를 uint8_t로 변환하는 함수
std::vector<uint8_t> convert_to_uint8(const std::vector<int64_t>& input) {
    std::vector<uint8_t> output;
    output.reserve(input.size());
    for (auto val : input) {
        output.push_back(static_cast<uint8_t>(val));  // int64_t를 uint8_t로 변환
    }
    return output;
}

// int64_t vector를 uint32_t로 변환하는 함수
std::vector<uint32_t> convert_to_uint32(const std::vector<int64_t>& input) {
    std::vector<uint32_t> output;
    output.reserve(input.size());
    for (auto val : input) {
        output.push_back(static_cast<uint32_t>(val));  // int64_t를 uint32_t로 변환
    }
    return output;
}
