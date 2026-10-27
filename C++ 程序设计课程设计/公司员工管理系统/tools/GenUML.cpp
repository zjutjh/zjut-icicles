#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <set>
#include <string>
#include <vector>

namespace {
    std::string ReadFileText(const std::filesystem::path &path) {
        std::ifstream input(path, std::ios::in | std::ios::binary);
        if (!input) {
            return {};
        }
        std::string content;
        input.seekg(0, std::ios::end);
        content.resize(static_cast<size_t>(input.tellg()));
        input.seekg(0, std::ios::beg);
        input.read(&content[0], static_cast<std::streamsize>(content.size()));
        return content;
    }

    bool IsHeaderFile(const std::filesystem::path &path) {
        auto ext = path.extension().string();
        for (auto &ch : ext) {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
        return ext == ".h" || ext == ".hpp" || ext == ".hh" || ext == ".hxx";
    }

    std::filesystem::path ResolveProjectRoot() {
        auto cwd = std::filesystem::current_path();
        if (cwd.filename() == "tools") {
            return cwd.parent_path();
        }
        return cwd;
    }

    struct ClassInfo {
        std::string name;
        std::vector<std::string> members;
    };

    std::string Trim(const std::string &value) {
        size_t start = value.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) {
            return {};
        }
        size_t end = value.find_last_not_of(" \t\r\n");
        return value.substr(start, end - start + 1);
    }

    bool StartsWith(const std::string &value, const std::string &prefix) {
        return value.rfind(prefix, 0) == 0;
    }

    std::string StripComments(const std::string &content) {
        std::string out;
        out.reserve(content.size());
        bool inLineComment = false;
        bool inBlockComment = false;
        bool inString = false;
        bool inChar = false;

        for (size_t i = 0; i < content.size(); ++i) {
            char c = content[i];
            char next = (i + 1 < content.size()) ? content[i + 1] : '\0';

            if (inLineComment) {
                if (c == '\n') {
                    inLineComment = false;
                    out.push_back(c);
                }
                continue;
            }
            if (inBlockComment) {
                if (c == '*' && next == '/') {
                    inBlockComment = false;
                    ++i;
                }
                continue;
            }

            if (!inString && !inChar) {
                if (c == '/' && next == '/') {
                    inLineComment = true;
                    ++i;
                    continue;
                }
                if (c == '/' && next == '*') {
                    inBlockComment = true;
                    ++i;
                    continue;
                }
            }

            if (!inChar && c == '"') {
                inString = !inString;
            } else if (!inString && c == '\'') {
                inChar = !inChar;
            }

            out.push_back(c);
        }

        return out;
    }

    bool IsTemplateParamClass(const std::string &content, size_t matchPos) {
        if (matchPos == 0 || content.empty()) {
            return false;
        }

        size_t lineStart = content.rfind('\n', matchPos);
        if (lineStart == std::string::npos) {
            lineStart = 0;
        }

        size_t windowStart = lineStart > 200 ? lineStart - 200 : 0;
        size_t templatePos = content.rfind("template", matchPos);
        if (templatePos == std::string::npos || templatePos < windowStart) {
            return false;
        }

        size_t angleStart = content.find('<', templatePos);
        if (angleStart == std::string::npos || angleStart > matchPos) {
            return false;
        }

        size_t angleEnd = content.find('>', angleStart);
        if (angleEnd == std::string::npos || angleEnd > matchPos) {
            return true;
        }

        return false;
    }

    bool IsEnumClassMatch(const std::string &content, size_t matchPos) {
        if (matchPos == 0 || content.empty()) {
            return false;
        }

        size_t pos = matchPos;
        while (pos > 0 && std::isspace(static_cast<unsigned char>(content[pos - 1]))) {
            --pos;
        }
        if (pos == 0) {
            return false;
        }

        size_t end = pos;
        size_t start = end;
        while (start > 0 && std::isalpha(static_cast<unsigned char>(content[start - 1]))) {
            --start;
        }

        if (start == end) {
            return false;
        }

        std::string word = content.substr(start, end - start);
        return word == "enum";
    }

    size_t FindTopLevelParen(const std::string &value) {
        int angleDepth = 0;
        for (size_t i = 0; i < value.size(); ++i) {
            char c = value[i];
            if (c == '<') {
                ++angleDepth;
            } else if (c == '>') {
                if (angleDepth > 0) {
                    --angleDepth;
                }
            } else if (c == '(' && angleDepth == 0) {
                return i;
            }
        }
        return std::string::npos;
    }

    std::string AccessSymbol(const std::string &access) {
        if (access == "public") {
            return "+";
        }
        if (access == "protected") {
            return "#";
        }
        return "-";
    }

    std::string ExtractMethodName(const std::string &prefix) {
        size_t opPos = prefix.rfind("operator");
        if (opPos != std::string::npos) {
            return Trim(prefix.substr(opPos));
        }

        size_t end = prefix.find_last_not_of(" \t");
        if (end == std::string::npos) {
            return {};
        }
        size_t start = prefix.find_last_of(" \t", end);
        if (start == std::string::npos) {
            start = 0;
        } else {
            ++start;
        }
        return prefix.substr(start, end - start + 1);
    }

    std::string ExtractReturnType(const std::string &prefix, const std::string &methodName) {
        size_t namePos = prefix.rfind(methodName);
        if (namePos == std::string::npos) {
            return {};
        }
        return Trim(prefix.substr(0, namePos));
    }

    bool IsValidIdentifier(const std::string &value) {
        if (value.empty()) {
            return false;
        }
        if (!std::isalpha(static_cast<unsigned char>(value[0])) && value[0] != '_') {
            return false;
        }
        for (size_t i = 1; i < value.size(); ++i) {
            char c = value[i];
            if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
                return false;
            }
        }
        return true;
    }

    std::string StripTrailingMethodQualifiers(std::string value) {
        value = Trim(value);
        if (value.empty()) {
            return value;
        }

        size_t rparenPos = value.find_last_of(')');
        size_t eqPos = value.find('=', rparenPos == std::string::npos ? 0 : rparenPos + 1);
        if (rparenPos != std::string::npos && eqPos != std::string::npos) {
            value = Trim(value.substr(0, eqPos));
        }

        static const std::vector<std::string> suffixes = {
            "override",
            "final",
            "noexcept"};

        bool trimmed = true;
        while (trimmed) {
            trimmed = false;
            for (const auto &suffix : suffixes) {
                if (value.size() >= suffix.size()) {
                    size_t pos = value.rfind(suffix);
                    if (pos != std::string::npos && pos + suffix.size() == value.size() &&
                        (pos == 0 || std::isspace(static_cast<unsigned char>(value[pos - 1])))) {
                        value = Trim(value.substr(0, pos));
                        trimmed = true;
                        break;
                    }
                }
            }
        }

        return value;
    }

    std::string StripInitializerList(const std::string &value) {
        int angleDepth = 0;
        int parenDepth = 0;
        for (size_t i = 0; i < value.size(); ++i) {
            char c = value[i];
            if (c == '<') {
                ++angleDepth;
            } else if (c == '>') {
                if (angleDepth > 0) {
                    --angleDepth;
                }
            } else if (c == '(') {
                ++parenDepth;
            } else if (c == ')' && parenDepth > 0) {
                --parenDepth;
            } else if (c == ':' && angleDepth == 0 && parenDepth == 0) {
                size_t j = i;
                while (j > 0 && std::isspace(static_cast<unsigned char>(value[j - 1]))) {
                    --j;
                }
                if (j > 0 && value[j - 1] == ')') {
                    return Trim(value.substr(0, i));
                }
            }
        }
        return value;
    }

    std::string NormalizeWhitespace(const std::string &value) {
        std::string out;
        out.reserve(value.size());
        bool inSpace = false;
        for (char c : value) {
            if (std::isspace(static_cast<unsigned char>(c))) {
                if (!inSpace) {
                    out.push_back(' ');
                    inSpace = true;
                }
            } else {
                out.push_back(c);
                inSpace = false;
            }
        }
        return Trim(out);
    }

    std::string EscapeMermaidText(const std::string &value) {
        std::string out;
        out.reserve(value.size());
        for (char c : value) {
            if (c == '<') {
                out += "&lt;";
            } else if (c == '>') {
                out += "&gt;";
            } else {
                out.push_back(c);
            }
        }
        return out;
    }

    std::string StripDefaultParamValues(const std::string &value) {
        std::string out;
        out.reserve(value.size());
        int parenDepth = 0;
        int angleDepth = 0;
        bool skipping = false;

        for (size_t i = 0; i < value.size(); ++i) {
            char c = value[i];
            if (c == '<') {
                ++angleDepth;
            } else if (c == '>') {
                if (angleDepth > 0) {
                    --angleDepth;
                }
            } else if (c == '(') {
                ++parenDepth;
            } else if (c == ')' && parenDepth > 0) {
                --parenDepth;
                skipping = false;
            }

            if (parenDepth > 0) {
                if (!skipping && c == '=' && angleDepth == 0) {
                    skipping = true;
                    continue;
                }
                if (skipping) {
                    if (c == ',' && angleDepth == 0) {
                        skipping = false;
                        out.push_back(c);
                    }
                    continue;
                }
            }

            out.push_back(c);
        }

        return out;
    }

    std::vector<std::string> ParseMembers(const std::string &body, const std::string &className) {
        std::vector<std::string> members;
        std::string access = "private";
        std::string buffer;
        int depth = 0;
        int parenDepth = 0;
        bool hasTopLevelParen = false;

        auto flushDeclaration = [&](const std::string &decl) {
            std::string trimmed = Trim(decl);
            if (trimmed.empty()) {
                return;
            }

            if (trimmed == "public:" || trimmed == "private:" || trimmed == "protected:") {
                access = trimmed.substr(0, trimmed.size() - 1);
                return;
            }

            if (StartsWith(trimmed, "using ") || StartsWith(trimmed, "typedef ") || StartsWith(trimmed, "friend ")) {
                return;
            }

            if (StartsWith(trimmed, "enum ") || StartsWith(trimmed, "class ") ||
                StartsWith(trimmed, "struct ") || StartsWith(trimmed, "union ")) {
                return;
            }

            std::string declaration = trimmed;
            if (!declaration.empty() && declaration.back() == ';') {
                declaration.pop_back();
            }

            size_t assignPos = declaration.find('=');
            size_t parenPos = declaration.find('(');
            if (parenPos != std::string::npos) {
                if (assignPos != std::string::npos && assignPos < parenPos) {
                    parenPos = std::string::npos;
                }
            }

            if (parenPos != std::string::npos) {
                std::string prefix = Trim(declaration.substr(0, parenPos));
                if (prefix.empty()) {
                    return;
                }

                static const std::vector<std::string> qualifiers = {
                    "virtual", "static", "inline", "constexpr", "explicit", "friend"};
                for (const auto &qual : qualifiers) {
                    if (StartsWith(prefix, qual + " ")) {
                        prefix = Trim(prefix.substr(qual.size()));
                        break;
                    }
                }

                std::string methodName = ExtractMethodName(prefix);
                if (methodName.empty()) {
                    return;
                }
                if (!StartsWith(methodName, "operator") && !IsValidIdentifier(methodName) &&
                    methodName != className && methodName != ("~" + className)) {
                    return;
                }

                if (methodName == ("~" + className)) {
                    methodName = "dtor";
                }
                if (methodName == className) {
                    methodName = className;
                }

                std::string signature = StripInitializerList(declaration);
                signature = StripTrailingMethodQualifiers(signature);
                signature = StripDefaultParamValues(signature);
                signature = NormalizeWhitespace(signature);
                if (signature.empty()) {
                    return;
                }

                std::string line = AccessSymbol(access) + EscapeMermaidText(signature) + "\n";
                members.push_back(line);
                return;
            }

            if (assignPos != std::string::npos) {
                declaration = Trim(declaration.substr(0, assignPos));
            }

            size_t nameEnd = declaration.find_last_not_of(" \t");
            if (nameEnd == std::string::npos) {
                return;
            }

            size_t nameStart = declaration.find_last_of(" \t", nameEnd);
            if (nameStart == std::string::npos) {
                nameStart = 0;
            } else {
                ++nameStart;
            }

            std::string name = declaration.substr(nameStart, nameEnd - nameStart + 1);
            std::string type = Trim(declaration.substr(0, nameStart));
            if (name.empty() || type.empty() || !IsValidIdentifier(name)) {
                return;
            }

            std::string line = AccessSymbol(access) +
                               EscapeMermaidText(NormalizeWhitespace(type + " " + name)) + "\n";
            members.push_back(line);
        };

        for (size_t i = 0; i < body.size(); ++i) {
            char c = body[i];

            if (depth == 0) {
                buffer.push_back(c);

                if (c == '(') {
                    ++parenDepth;
                } else if (c == ')' && parenDepth > 0) {
                    --parenDepth;
                    if (parenDepth == 0) {
                        hasTopLevelParen = true;
                    }
                }

                if (c == ':') {
                    std::string maybeAccess = Trim(buffer);
                    if (maybeAccess == "public:" || maybeAccess == "private:" || maybeAccess == "protected:") {
                        flushDeclaration(maybeAccess);
                        buffer.clear();
                    }
                }

                if (c == ';') {
                    flushDeclaration(buffer);
                    buffer.clear();
                    parenDepth = 0;
                    hasTopLevelParen = false;
                } else if (c == '{') {
                    if (hasTopLevelParen) {
                        std::string decl = Trim(buffer.substr(0, buffer.size() - 1));
                        if (!decl.empty()) {
                            flushDeclaration(decl + ";");
                        }
                        buffer.clear();
                        depth = 1;
                        parenDepth = 0;
                        hasTopLevelParen = false;
                    }
                }
            } else {
                if (c == '{') {
                    ++depth;
                } else if (c == '}') {
                    --depth;
                }
            }
        }

        return members;
    }

    std::vector<ClassInfo> ExtractClasses(const std::string &content) {
        std::vector<ClassInfo> classes;
        static const std::regex classRegex(R"(\bclass\s+([A-Za-z_][A-Za-z0-9_]*))");

        for (std::sregex_iterator it(content.begin(), content.end(), classRegex), end; it != end; ++it) {
            size_t searchPos = static_cast<size_t>((*it).position());
            if (IsTemplateParamClass(content, searchPos) || IsEnumClassMatch(content, searchPos)) {
                continue;
            }
            const std::string className = (*it)[1].str();
            size_t bracePos = content.find('{', searchPos);
            if (bracePos == std::string::npos) {
                classes.push_back({className, {}});
                continue;
            }

            size_t pos = bracePos + 1;
            int depth = 1;
            size_t bodyStart = pos;
            while (pos < content.size() && depth > 0) {
                if (content[pos] == '{') {
                    ++depth;
                } else if (content[pos] == '}') {
                    --depth;
                }
                ++pos;
            }

            if (depth != 0 || pos <= bodyStart) {
                classes.push_back({className, {}});
                continue;
            }

            const std::string body = content.substr(bodyStart, pos - bodyStart - 1);
            std::vector<std::string> members = ParseMembers(body, className);

            classes.push_back({className, members});
        }

        return classes;
    }

    struct EnumInfo {
        std::string name;
        std::vector<std::string> values;
    };

    std::vector<EnumInfo> ExtractEnumClasses(const std::string &content) {
        std::vector<EnumInfo> enums;
        static const std::regex enumRegex(R"(\benum\s+class\s+([A-Za-z_][A-Za-z0-9_]*))");

        for (std::sregex_iterator it(content.begin(), content.end(), enumRegex), end; it != end; ++it) {
            const std::string enumName = (*it)[1].str();
            size_t searchPos = static_cast<size_t>((*it).position());
            size_t bracePos = content.find('{', searchPos);
            if (bracePos == std::string::npos) {
                enums.push_back({enumName, {}});
                continue;
            }

            size_t pos = bracePos + 1;
            int depth = 1;
            size_t bodyStart = pos;
            while (pos < content.size() && depth > 0) {
                if (content[pos] == '{') {
                    ++depth;
                } else if (content[pos] == '}') {
                    --depth;
                }
                ++pos;
            }

            if (depth != 0 || pos <= bodyStart) {
                enums.push_back({enumName, {}});
                continue;
            }

            std::string body = content.substr(bodyStart, pos - bodyStart - 1);
            std::vector<std::string> values;
            std::string token;

            for (size_t i = 0; i <= body.size(); ++i) {
                char c = i < body.size() ? body[i] : ',';
                if (c == ',') {
                    std::string item = Trim(token);
                    token.clear();
                    if (item.empty()) {
                        continue;
                    }
                    size_t eqPos = item.find('=');
                    if (eqPos != std::string::npos) {
                        item = Trim(item.substr(0, eqPos));
                    }
                    if (!item.empty()) {
                        values.push_back(item);
                    }
                } else {
                    token.push_back(c);
                }
            }

            enums.push_back({enumName, values});
        }

        return enums;
    }
} // namespace

int main(int argc, char **argv) {
    std::filesystem::path projectRoot = ResolveProjectRoot();
    std::filesystem::path srcPath = projectRoot / "src";
    std::filesystem::path outputPath = projectRoot / "tools/Result-GenUML.md";

    if (argc >= 2) {
        srcPath = argv[1];
    }
    if (argc >= 3) {
        outputPath = argv[2];
    }

    if (!std::filesystem::exists(srcPath)) {
        std::cerr << "Source path does not exist: " << srcPath.string() << "\n";
        return 1;
    }

    std::set<std::string> uniqueClasses;
    std::set<std::string> uniqueEnums;
    struct ClassRecord {
        std::string filePath;
        ClassInfo info;
    };
    std::vector<ClassRecord> classRecords;
    struct EnumRecord {
        std::string filePath;
        EnumInfo info;
    };
    std::vector<EnumRecord> enumRecords;

    for (const auto &entry : std::filesystem::recursive_directory_iterator(srcPath)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto &path = entry.path();
        if (!IsHeaderFile(path)) {
            continue;
        }

        std::string content = ReadFileText(path);
        if (content.empty()) {
            continue;
        }

        std::string cleanContent = StripComments(content);

        for (const auto &classInfo : ExtractClasses(cleanContent)) {
            if (uniqueClasses.insert(classInfo.name).second) {
                classRecords.push_back({path.string(), classInfo});
            }
        }

        for (const auto &enumInfo : ExtractEnumClasses(cleanContent)) {
            if (uniqueEnums.insert(enumInfo.name).second) {
                enumRecords.push_back({path.string(), enumInfo});
            }
        }
    }

    std::ofstream output(outputPath, std::ios::out | std::ios::trunc);
    if (!output) {
        std::cerr << "Failed to write output: " << outputPath.string() << "\n";
        return 1;
    }

    output << "# Mermaid Class Diagram Blocks\n\n";
    if (classRecords.empty()) {
        output << "No classes found.\n";
        return 0;
    }

    for (const auto &record : classRecords) {
        output << "## " << record.info.name << "\n";
        output << "Source: " << record.filePath << "\n\n";
        output << "```mermaid\n";
        output << "classDiagram\n";
        output << "class " << record.info.name << " {\n";
        for (const auto &member : record.info.members) {
            output << "  " << member;
        }
        output << "}\n";
        output << "```\n\n";
    }

    for (const auto &record : enumRecords) {
        output << "## " << record.info.name << "\n";
        output << "Source: " << record.filePath << "\n\n";
        output << "```mermaid\n";
        output << "classDiagram\n";
        output << "class " << record.info.name << " {\n";
        output << "  <<enumeration>>\n";
        for (const auto &value : record.info.values) {
            output << "  " << value << "\n";
        }
        output << "}\n";
        output << "```\n\n";
    }

    std::cout << "Generated: " << outputPath.string() << "\n";
    return 0;
}
