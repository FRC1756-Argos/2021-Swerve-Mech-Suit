#pragma once

// Helper function generator to detect if a namespace has a member defined
#define HAS_MEMBER(X) \
  template <typename T, typename = void> \
  struct has_##X : std::false_type{}; \
  template <typename T> \
  struct has_##X<T, decltype((void)T::X, void())> : std::true_type {};
