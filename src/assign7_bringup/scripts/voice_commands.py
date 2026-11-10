"""Conservative Vietnamese navigation grammar, independent of ASR and ROS."""
import re
import unicodedata


class CommandError(ValueError):
    pass


def normalize(text):
    text = str(text).lower().replace('đ', 'd')
    text = ''.join(c for c in unicodedata.normalize('NFKD', text) if not unicodedata.combining(c))
    return ' '.join(re.sub(r'[^a-z0-9\s]', ' ', text).split())


def parse_command(text, regions):
    """Return a fully resolved ordered route, [] for stop, or raise (never a partial route)."""
    text = normalize(text)
    if re.fullmatch(r'(?:robot )?(?:dung(?: lai| robot)?|huy(?: lenh| dieu huong)?)(?: ngay)?', text):
        return []
    match = re.fullmatch(r'(?:robot )?(?:hay )?(?:di chuyen|di|den|toi)\s+(.+)', text)
    if not match:
        raise CommandError('Không phải lệnh điều hướng. Ví dụ: Đi đến phòng học.')
    rest = re.sub(r'\s+(?:nhe|nha|giup toi)$', '', match.group(1))
    chunks = [s.strip() for s in re.split(r'\b(?:roi den|sau do den|den|toi|qua|tu|va)\b', rest) if s.strip()]
    if not chunks or (rest.startswith('tu ') and len(chunks) < 2):
        raise CommandError('Thiếu khu vực đích hoặc lệnh từ A đến B chưa đầy đủ.')
    if len(chunks) > 5:
        raise CommandError('Chỉ nhận tối đa 5 khu vực trong một lệnh.')
    aliases = {}
    for region in regions:
        for name in [region['id'], region['name'], *region.get('aliases', [])]:
            alias = normalize(name)
            if alias in aliases and aliases[alias] != region['id']:
                raise CommandError(f'Tên khu vực trùng: {name}')
            aliases[alias] = region['id']
    route = []
    for chunk in chunks:
        if chunk not in aliases:
            raise CommandError(f'Khu vực không tồn tại hoặc chưa rõ: "{chunk}". Không gửi goal.')
        if not route or route[-1] != aliases[chunk]:
            route.append(aliases[chunk])
    return route
