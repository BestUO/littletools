#ifndef DIALOGMANAGER_TEXTCOMPARE_H
#define DIALOGMANAGER_TEXTCOMPARE_H

#include <string>
#include <vector>


using MatchResult = std::pair<std::string, float>;

//define match strategy
enum class MatchStrategy {
    WHOLE = 0, //全语句匹配
    SLICE,  //分词匹配，分词的大小由OC配置文件决定
    SEMANTIC //语义匹配
};

using StringArray = std::vector<std::string>;

class TextCompare {
public:
    TextCompare();

    virtual float GetCompareResult(const std::string &left,
                                   const std::string &right, const MatchStrategy &strategy);

    void SetStopWords(const std::string &stop_words);

    void SetSymbols(const std::string &symbols);

    void SetAdvanceWhenHit(float value);

    void SetUseKeywordMatch(bool value);

private:
    static StringArray SplitWord(const std::string &input);

    static float EditDistance(const StringArray &left_chars,
                              const StringArray &right_chars);

    static float Sigmoid(float input);

    static float CosSimilarity(const StringArray &left_chars,
                               const StringArray &right_chars);

    static float Smooth(float cos, float distance);

    static float SimilaritySmooth(float cos, float factor1,
                                  float distance, float factor2);

    static size_t Count(const std::string &src, const std::string &sub);

    void CleanSentence(StringArray &chars);

    static constexpr size_t kSizePerWord = 3;
    //分词窗口的大小, 0 则代表不进行分词
    int slice_size_;
    float advance_when_hit_;
    std::string stop_words_;
    std::string symbols_;
    bool use_keyword_match_;
};


#endif //DIALOGMANAGER_TEXTCOMPARE_H
