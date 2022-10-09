#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

int factorial(int number) { return number <= 1 ? number : factorial(number - 1) * number; }

TEST_CASE("testing the factorial function") {
    CHECK(factorial(1) == 1);
    CHECK(factorial(2) == 2);
    CHECK(factorial(3) == 6);
    CHECK(factorial(10) == 3628800);
}

TEST_CASE("vectors can be sized and resized") {
    std::vector<int> v(5);

    REQUIRE(v.size() == 5);
    REQUIRE(v.capacity() >= 5);

    SUBCASE("adding to the vector increases it's size") {
        v.push_back(1);

        CHECK(v.size() == 6);
        CHECK(v.capacity() >= 6);
    }
    SUBCASE("reserving increases just the capacity") {
        v.reserve(6);
        std::cout << "2" << std::endl;
        CHECK(v.size() == 5);
        CHECK(v.capacity() >= 6);
        REQUIRE_FALSE(false);
        SUBCASE("") { std::cout << "2.1" << std::endl; }
        MESSAGE(std::to_string(2));
    }
}

TEST_CASE_TEMPLATE("signed integers stuff", T, signed char, float, int) {
    T var = T();
    --var;
    CHECK(var == -1);
}

TEST_CASE_TEMPLATE("vector stuff", T, std::vector<int>) {
    T vec(10);

    CHECK(vec.size() == 20); // will fail
}

TEST_CASE("200ms limit" * doctest::timeout(0.2)) {
    // sleep(1);
}

template <typename first, typename second>
struct TypePair
{
    typedef first  A;
    typedef second B;
};
TEST_CASE_TEMPLATE("multiple types", T, TypePair<int, char>, TypePair<char, int>, TypePair<bool, int>) {
    using T1 = typename T::A;
    using T2 = typename T::B;
    T1 t1 = T1();
    T2 t2 = T2();
    // use T1 and T2 types
    CHECK(t1 == T1());
    CHECK(t2 != T2());
}
