
#ifndef RAPIDJSON_CURSORSTREAMWRAPPER_H_
#define RAPIDJSON_CURSORSTREAMWRAPPER_H_

#include "stream.h"

#if defined(__GNUC__)
RAPIDJSON_DIAG_PUSH
RAPIDJSON_DIAG_OFF(effc++)
#endif

#if defined(_MSC_VER) && _MSC_VER <= 1800
RAPIDJSON_DIAG_PUSH
RAPIDJSON_DIAG_OFF(4702)  // unreachable code
RAPIDJSON_DIAG_OFF(4512)  // assignment operator could not be generated
#endif

RAPIDJSON_NAMESPACE_BEGIN

//! Cursor stream wrapper for counting line and column number if error exists.
/*!
    \tparam InputStream     Any stream that implements Stream Concept
*/
template <typename InputStream, typename Encoding = UTF8<>>
class CursorStreamWrapper : public GenericStreamWrapper<InputStream, Encoding> {
 public:
  typedef typename Encoding::Ch Ch;

  CursorStreamWrapper(InputStream &is)
      : GenericStreamWrapper<InputStream, Encoding>(is), line_(1), col_(0) {}

  // counting line and column number
  Ch Take() {
    Ch ch = this->is_.Take();
    if (ch == '\n') {
      line_++;
      col_ = 0;
    } else {
      col_++;
    }
    return ch;
  }

  //! Get the error line number, if error exists.
  size_t GetLine() const { return line_; }
  //! Get the error column number, if error exists.
  size_t GetColumn() const { return col_; }

 private:
  size_t line_;  //!< Current Line
  size_t col_;   //!< Current Column
};

#if defined(_MSC_VER) && _MSC_VER <= 1800
RAPIDJSON_DIAG_POP
#endif

#if defined(__GNUC__)
RAPIDJSON_DIAG_POP
#endif

RAPIDJSON_NAMESPACE_END

#endif  // RAPIDJSON_CURSORSTREAMWRAPPER_H_
