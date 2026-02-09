template <typename Out, typename In>
bool check_r(Out* out, In value, In min, In max)
{
    // Range check happens in the signed type (In)
    if (value < min) return false;
    if (value > max) return false;

    // Only cast AFTER validation
    *out = (Out)value;
    return true;
}
