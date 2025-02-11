import React from 'react';
import NodeExpanded from './NodeExpanded';
import MiniSearch from 'minisearch';
import hangul from 'hangul-js';
import styled from 'styled-components';
import NodeHandler from './NodeHandler';
import {
  Container,
  Rows,
  HistoryCount,
  Title
} from './ContainerStyles';

const TitleExpanded = styled(Title)`
  color: yellow;
`
const SEARCH_OPTION = {
  prefix: true,
  fields: ['json.title'],
  fuzzy: 0.3
};

function ExpandedContainer(props) {
  // eslint-disable-next-line react/prop-types
  const {
    checkedNodeList,
    setCheckedNodeList,
    nodesExpanded, 
    removeNode, 
    expandNode,
    shrinkNode,
    lastNetworkData,
    setLastNetworkData,
    setBacklinksToShow,
    setForwardlinksToShow,
    setActiveExpandedNodeId,
    addCheckedNodeList,
    delCheckedNodeList,
    focusNode
  } = props;
  const miniSearchRef = React.useRef(null);

  const searchTitle = React.useCallback((title) => {
    const searchPattern =
      hangul.disassemble(title).join('') || MiniSearch.wildcard;
    return miniSearchRef.current.search(searchPattern, SEARCH_OPTION);
  }, []);

  return (
    <Container>
      <TitleExpanded>Node Expanded</TitleExpanded>
      <Rows>
        {nodesExpanded.map((node) => (
          <NodeExpanded
            key={node.id}
            node={node}
            checkedNodeList={checkedNodeList}
            removeNode={removeNode}
            lastNetworkData={lastNetworkData}
            setBacklinksToShow={setBacklinksToShow}
            setForwardlinksToShow={setForwardlinksToShow}
            setLastNetworkData={setLastNetworkData}
            setActiveExpandedNodeId={setActiveExpandedNodeId}
            addCheckedNodeList={addCheckedNodeList}
            delCheckedNodeList={delCheckedNodeList}
            focusNode={focusNode}
          ></NodeExpanded>
        ))}
      </Rows>
      <HistoryCount>
        {nodesExpanded.length} shown
      </HistoryCount>
      <NodeHandler
        checkedNodeList={checkedNodeList}
        setCheckedNodeList={setCheckedNodeList}
        removeNode={removeNode}
        expandNode={expandNode}
        shrinkNode={shrinkNode}
      ></NodeHandler>
    </Container>
  )
}

export default React.memo(ExpandedContainer);