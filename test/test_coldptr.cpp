#include <gtest/gtest.h>

#include <type_traits>
#include "ColdPtr.hpp"

template <class...>
using void_t = void;

// Data structure for tests
struct Data { int x{}, y{}; };

class C {
    struct Ptr : public ColdPtr<Data> {
        friend class C;
        using ColdPtr<Data>::ColdPtr;
    };

public:
    enum { valX = 2, valY = 3 };

private:
    Data m_data;
    Ptr m_ptr;

public:
    static_assert(!std::is_same<decltype(*std::declval<Ptr>()), Data const&>::value, __FILE__);
    static_assert(!std::is_same<decltype(std::declval<Ptr>()->x), int&>::value, __FILE__);

    C() : m_data{valX,valY}, m_ptr{&m_data} {}
    Data get() { return *m_ptr; }
    int getX() { return m_ptr->x; }

    Ptr getPtr() { return m_ptr; }
};

TEST(ColdPtr, TraitsPublic)
{
    static_assert(std::is_literal_type<ColdPtr<Data>>::value, __FILE__);
    static_assert(!std::is_constructible<ColdPtr<Data>,Data*>::value, __FILE__);
    static_assert(std::is_copy_constructible<ColdPtr<Data>>::value, __FILE__);
    static_assert(std::is_copy_assignable<ColdPtr<Data>>::value, __FILE__);
    static_assert(std::is_default_constructible<ColdPtr<Data>>::value, __FILE__);
    static_assert(std::is_move_constructible<ColdPtr<Data>>::value, __FILE__);
    static_assert(std::is_move_assignable<ColdPtr<Data>>::value, __FILE__);

    /* The dereferencing operator and the arrow operator are not available.
    static_assert(!std::is_same<decltype(*std::declval<ColdPtr<Data>>()), Data&>::value, __FILE__);
    static_assert(!std::is_same<decltype(std::declval<ColdPtr<Data>>()->x), int>::value, __FILE__);
    */
}

TEST(ColdPtr, TraitsFriend)
{
    C var;
    EXPECT_EQ(var.get().x, C::valX);
    EXPECT_EQ(var.get().y, C::valY);
    EXPECT_EQ(var.getX(), C::valX);
}

TEST(ColdPtr, Validity)
{
    ColdPtr<int> ptr1;

    EXPECT_FALSE(static_cast<bool>(ptr1));

    C var;
    auto ptr = var.getPtr();

    EXPECT_TRUE(static_cast<bool>(ptr));
    EXPECT_FALSE(!ptr);

    ptr.invalidate();

    EXPECT_FALSE(static_cast<bool>(ptr));
    EXPECT_TRUE(!ptr);
}
